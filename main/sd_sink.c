/* sd_sink.c — see sd_sink.h. */
#include "sd_sink.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "app_state.h"
#include "config.h"

#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG = "sd";

static sdmmc_card_t *s_card;
static FILE *s_file;
static char s_path[32];         /* current session file (for close-verify) */
static uint32_t s_written;      /* fwrite-confirmed bytes this session */
static uint32_t s_dropped;      /* short/failed writes this session */

esp_err_t sd_refresh_stats(void)
{
    if (!s_card) return ESP_ERR_INVALID_STATE;
    FATFS *fs;
    DWORD fre_clust = 0;
    FRESULT fr = f_getfree("0:", &fre_clust, &fs);   /* "0:" == /sdcard */
    if (fr != FR_OK) {
        g_app.sd_err++;
        ESP_LOGW(TAG, "f_getfree failed (%d)", (int)fr);
        return ESP_FAIL;
    }
    g_app.sd_total_bytes = (uint64_t)s_card->csd.capacity * 512;
    g_app.sd_free_bytes = (uint64_t)fre_clust * fs->csize * 512;
    return ESP_OK;
}

esp_err_t sd_init(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;
    host.flags = SDMMC_HOST_FLAG_1BIT;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk = SD_PIN_CLK;
    slot.cmd = SD_PIN_CMD;
    slot.d0 = SD_PIN_D0;
    slot.width = 1;

    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = 2,
        .allocation_unit_size = 16 * 1024,
    };
    esp_err_t err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount,
                                            &s_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no SD card (mount failed: %s) — USB-only mode",
                 esp_err_to_name(err));
        g_app.sd_present = 0;
        g_app.sd_total_bytes = g_app.sd_free_bytes = 0;
        return err;                       /* absent card is NOT fatal */
    }
    g_app.sd_present = 1;
    sdmmc_card_print_info(stdout, s_card);
    sd_refresh_stats();
    return ESP_OK;
}

/* scan the card for existing session files; returns the highest LOG
 * number found (0 when none).  Case-insensitive on the name body: FAT
 * short names are created uppercase by us but a card moved between
 * systems may carry lowercase entries.  sscanf %5u caps the numeric
 * field at 5 digits; a longer stem makes the trailing literal mismatch
 * and the entry is ignored (harmless — O_EXCL still protects). */
static uint32_t sd_scan_max_id(void)
{
    uint32_t max_id = 0;
    DIR *d = opendir("/sdcard");
    if (!d) {
        g_app.sd_err++;
        ESP_LOGW(TAG, "opendir /sdcard failed (%s)", strerror(errno));
        return 0;
    }
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        unsigned n;
        if ((sscanf(e->d_name, "LOG%5u.CSV", &n) == 1 ||
             sscanf(e->d_name, "log%5u.csv", &n) == 1) && n > max_id)
            max_id = n;
    }
    closedir(d);
    return max_id;
}

esp_err_t sd_open_session_auto(uint32_t *session_id)
{
    if (!g_app.sd_present || !s_card) return ESP_ERR_INVALID_STATE;
    uint32_t base = sd_scan_max_id();

    /* O_CREAT|O_EXCL maps to FatFS FA_CREATE_NEW in the IDF fatfs VFS:
     * creation fails with EEXIST rather than truncating when the name
     * exists, so a scan race degrades to the next candidate. */
    for (int tries = 0; tries < 1000; tries++) {
        uint32_t cand = base + 1 + (uint32_t)tries;
        snprintf(s_path, sizeof(s_path), "/sdcard/LOG%05lu.CSV",
                 (unsigned long)cand);
        int fd = open(s_path, O_WRONLY | O_CREAT | O_EXCL, 0666);
        if (fd < 0) {
            if (errno == EEXIST) continue;      /* raced: try next number */
            ESP_LOGE(TAG, "open %s failed: %s", s_path, strerror(errno));
            g_app.sd_err++;
            return ESP_FAIL;
        }
        s_file = fdopen(fd, "w");
        if (!s_file) {
            ESP_LOGE(TAG, "fdopen %s failed: %s", s_path, strerror(errno));
            close(fd);
            g_app.sd_err++;
            return ESP_FAIL;
        }
        s_written = s_dropped = 0;
        *session_id = cand;
        ESP_LOGI(TAG, "session %lu -> %s (exclusive create)",
                 (unsigned long)cand, s_path);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "no free session name after 1000 tries");
    g_app.sd_err++;
    return ESP_FAIL;
}

int sd_session_open(void) { return s_file != NULL; }
uint32_t sd_session_bytes(void) { return s_written; }
uint32_t sd_session_drops(void) { return s_dropped; }

uint64_t sd_free_now(void)
{
    uint64_t f = g_app.sd_free_bytes;
    if (s_file && f >= s_written) f -= s_written;
    return f;
}

void sd_write(const char *line, size_t len)
{
    if (!s_file) return;
    if (fwrite(line, 1, len, s_file) != len) {
        s_dropped++;
        g_app.sd_err++;                 /* visible on the Status screen */
        return;
    }
    s_written += (uint32_t)len;
    /* flush every ~8 KB so a power loss loses at most a small tail */
    if ((s_written % 8192) < len)
        if (fflush(s_file) != 0)
            g_app.sd_err++;
}

esp_err_t sd_close_session(void)
{
    if (!s_file) return ESP_OK;
    int bad = 0;
    if (fflush(s_file) != 0) bad = 1;
    if (fclose(s_file) != 0) bad = 1;
    s_file = NULL;
    if (bad) {
        g_app.sd_err++;
        ESP_LOGE(TAG, "session close/flush failed (%lu bytes written)",
                 (unsigned long)s_written);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "session closed (%lu bytes, %lu dropped writes)",
             (unsigned long)s_written, (unsigned long)s_dropped);

    /* normal-close verification: read the file back and confirm the
     * START marker is the first line and the STOP marker the last.
     * Interrupted (power-loss) sessions are verified never to occur:
     * they simply keep their partial data and the next boot's scan
     * guarantees the follow-up session picks a fresh name. */
    FILE *f = fopen(s_path, "r");
    if (!f) {
        g_app.sd_err++;
        ESP_LOGE(TAG, "verify: cannot reopen %s", s_path);
    } else {
        char ln[160], first[160] = "";
        char last[160] = "";
        uint32_t lines = 0;
        while (fgets(ln, sizeof(ln), f)) {
            if (lines == 0) {
                strncpy(first, ln, sizeof(first) - 1);
                first[sizeof(first) - 1] = 0;
            }
            strncpy(last, ln, sizeof(last) - 1);
            last[sizeof(last) - 1] = 0;
            lines++;
        }
        fclose(f);
        int first_ok = strncmp(first, "S,", 2) == 0 &&
                       strstr(first, ",1,") != NULL;
        int last_ok = strncmp(last, "S,", 2) == 0 &&
                      strstr(last, ",0,") != NULL;
        if (lines < 3 || !first_ok || !last_ok) {
            g_app.sd_err++;
            ESP_LOGE(TAG, "verify FAIL: %lu lines, first='%s' last='%s'",
                     (unsigned long)lines, first, last);
        } else {
            ESP_LOGI(TAG, "verify OK: %lu lines, START + STOP markers",
                     (unsigned long)lines);
        }
    }
    sd_refresh_stats();                 /* free space: real again */
    return ESP_OK;
}
