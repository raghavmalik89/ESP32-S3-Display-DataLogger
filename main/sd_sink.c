/* sd_sink.c — see sd_sink.h. */
#include "sd_sink.h"

#include <stdio.h>
#include <string.h>

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

esp_err_t sd_open_session(uint32_t session_id)
{
    if (!g_app.sd_present || !s_card) return ESP_ERR_INVALID_STATE;
    char path[32];
    snprintf(path, sizeof(path), "/sdcard/LOG%05lu.CSV",
             (unsigned long)session_id);
    s_file = fopen(path, "w");
    if (!s_file) {
        ESP_LOGE(TAG, "open %s failed", path);
        g_app.sd_err++;
        return ESP_FAIL;
    }
    s_written = s_dropped = 0;
    ESP_LOGI(TAG, "session %lu -> %s", (unsigned long)session_id, path);
    return ESP_OK;
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
    sd_refresh_stats();                 /* free space: real again */
    return ESP_OK;
}
