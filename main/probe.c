/* probe.c — hardware discovery app (CONFIG_LOGGER_PROBE_MODE=y).
 *
 * Clean dual-IMU diagnostic (QMI8658C onboard + external ST LSM6DS),
 * run after the CTRL8/CTRL9 register-map fix in qmi8658c.c:
 *   1. shared I2C bus scan on GPIO47/48
 *   2. identity reads at the IMU addresses: QMI WHO_AM_I (reg 0x00 ->
 *      0x05), QMI REVISION_ID (0x01), QMI CTRL1 (0x02), ST WHO_AM_I
 *      (reg 0x0F -> 0x70 LSM6DSV16X / 0x6C LSM6DSO / 0x69 LSM6DS3)
 *   3. driver-level init of both IMUs with address-conflict detection
 *      (ST answering on the address the QMI owns -> physical fix needed)
 *   4. LCD color fill smoke test
 *   5. buzzer tone sweep on GPIO16
 *   6. 15 s live-data loop: accel/gyro of each detected IMU at 1 Hz
 *
 * Earlier QMI-death experiments (bit-bang, legacy-speed sweep, register
 * poke sequence) were removed: the root cause was the swapped CTRL8/CTRL9
 * defines, fixed in qmi8658c.c.  History: docs/SENSORS.md, commit a0d0efc.
 */
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "i2c_bus.h"
#include "lcd.h"
#include "qmi8658c.h"
#include "bmp581.h"
#include "imu_lsm6dsv16x.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "probe";

/* LSM6DS-family WHO_AM_I values */
#define WHO_LSM6DSV16X   0x70
#define WHO_LSM6DSO      0x6C
#define WHO_LSM6DS3      0x69

static void probe_i2c(void)
{
    ESP_ERROR_CHECK(i2c_bus_init());
    ESP_LOGI(TAG, "I2C bus up: SDA=%d SCL=%d @ %d kHz", I2C_SDA_GPIO,
             I2C_SCL_GPIO, I2C_FREQ_HZ / 1000);

    /* 1. bus scan */
    char line[200];
    int n = snprintf(line, sizeof(line), "scan:");
    for (uint8_t a = 0x08; a < 0x78 && n < (int)sizeof(line) - 6; a++) {
        if (i2c_probe_addr(a))
            n += snprintf(line + n, sizeof(line) - n, " 0x%02x", a);
    }
    ESP_LOGI(TAG, "%s", line);

    /* 2. identity reads wherever something ACKs */
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (!i2c_probe_addr(a)) continue;
        i2c_master_dev_handle_t dev = NULL;
        i2c_device_config_t dc = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = a,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
        if (i2c_master_bus_add_device(g_i2c_bus, &dc, &dev) != ESP_OK)
            continue;

        if (a == 0x46 || a == 0x47) {
            uint8_t chipid = 0;
            if (i2c_read_reg8(dev, 0x01, &chipid) == ESP_OK)
                ESP_LOGI(TAG, "addr 0x%02x: BMP581-class CHIP_ID=0x%02x",
                         a, chipid);
        } else if (a == 0x6A || a == 0x6B) {
            /* QMI8658C identity: reg 0x00 -> 0x05; ST LSM6DS identity:
             * reg 0x0F -> family ID.  Reading both on every ACK'd
             * candidate exposes an address conflict (both chips on one
             * address garble each other's register file). */
            uint8_t qid = 0xAA, rev = 0xAA, c1 = 0xAA, sid = 0xAA;
            esp_err_t e_q = i2c_read_reg8(dev, 0x00, &qid);
            i2c_read_reg8(dev, 0x01, &rev);
            i2c_read_reg8(dev, 0x02, &c1);
            esp_err_t e_s = i2c_read_reg8(dev, 0x0F, &sid);
            ESP_LOGI(TAG, "addr 0x%02x: QMI reg0 err=%d val=0x%02x "
                     "REV=0x%02x CTRL1=0x%02x | ST reg0x0F err=%d val=0x%02x",
                     a, e_q, qid, rev, c1, e_s, sid);
        } else {
            ESP_LOGI(TAG, "addr 0x%02x: present, identity unknown", a);
        }
        i2c_master_bus_rm_device(dev);
    }
}

static void probe_buzzer(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t c = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 512,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ESP_ERROR_CHECK(ledc_channel_config(&c));
    ESP_LOGI(TAG, "buzzer sweep on GPIO%d ...", BUZZER_GPIO);
    for (int hz = 500; hz <= 3500; hz += 500) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, hz);
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "buzzer sweep done");
}

/* ---- LCD diagnostic: fills, border, corner markers, text ------------- */

static void draw_hline(int x, int y, int w, uint16_t c)
{
    static uint16_t buf[LCD_H_RES];
    for (int i = 0; i < w; i++) buf[i] = c;
    lcd_blit(x, y, w, 1, buf);
}

static void draw_vline(int x, int y, int h, uint16_t c)
{
    uint16_t buf[LCD_V_RES];
    for (int i = 0; i < h; i++) buf[i] = c;
    lcd_blit(x, y, 1, h, buf);
}

static void draw_block(int x, int y, int w, int h, uint16_t c)
{
    static uint16_t buf[16 * 16];
    for (int i = 0; i < w * h; i++) buf[i] = c;
    for (int r = 0; r < h; r++) lcd_blit(x, y + r, w, 1, buf);
}

static void probe_lcd_diag(void)
{
    /* full-screen color fills (each flushed immediately by lcd_fill) */
    lcd_fill(LCD_RED);    vTaskDelay(pdMS_TO_TICKS(600));
    lcd_fill(LCD_GREEN);  vTaskDelay(pdMS_TO_TICKS(600));
    lcd_fill(LCD_BLUE);   vTaskDelay(pdMS_TO_TICKS(600));
    lcd_fill(LCD_BLACK);

    /* white border, 2 px, full 320x170 logical area */
    draw_hline(0, 0, LCD_H_RES, LCD_WHITE);
    draw_hline(0, 1, LCD_H_RES, LCD_WHITE);
    draw_hline(0, LCD_V_RES - 2, LCD_H_RES, LCD_WHITE);
    draw_hline(0, LCD_V_RES - 1, LCD_H_RES, LCD_WHITE);
    draw_vline(0, 0, LCD_V_RES, LCD_WHITE);
    draw_vline(1, 0, LCD_V_RES, LCD_WHITE);
    draw_vline(LCD_H_RES - 2, 0, LCD_V_RES, LCD_WHITE);
    draw_vline(LCD_H_RES - 1, 0, LCD_V_RES, LCD_WHITE);

    /* corner markers + labels: TL red, TR green, BL blue, BR yellow */
    draw_block(4, 4, 12, 12, LCD_RED);
    draw_block(LCD_H_RES - 16, 4, 12, 12, LCD_GREEN);
    draw_block(4, LCD_V_RES - 16, 12, 12, LCD_BLUE);
    draw_block(LCD_H_RES - 16, LCD_V_RES - 16, 12, 12, LCD_YELLOW);
    lcd_text(20, 6, "TL", LCD_RED, LCD_BLACK);
    lcd_text(LCD_H_RES - 36, 6, "TR", LCD_GREEN, LCD_BLACK);
    lcd_text(20, LCD_V_RES - 12, "BL", LCD_BLUE, LCD_BLACK);
    lcd_text(LCD_H_RES - 36, LCD_V_RES - 12, "BR", LCD_YELLOW, LCD_BLACK);

    /* asymmetric text: must read left-to-right, upright */
    lcd_text(136, 56, "ABC123", LCD_WHITE, LCD_BLACK);
    lcd_text(116, 72, "LEFT->RIGHT", LCD_GREEN, LCD_BLACK);
    lcd_text(92, 88, "VARIO LOGGER REV3", LCD_CYAN, LCD_BLACK);
#if LCD_ROT_CW
    lcd_text(104, 120, "ROT CW", LCD_YELLOW, LCD_BLACK);
#else
    lcd_text(104, 120, "ROT CCW", LCD_YELLOW, LCD_BLACK);
#endif

    lcd_flush();
}

void probe_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));   /* let USB enumerate */
    vTaskDelay(pdMS_TO_TICKS(4000));  /* let the host serial attach before
                                       * the scan results are printed */
    ESP_LOGI(TAG, "=== sensor-logger probe v2 (post CTRL8/CTRL9 fix) ===");

    probe_i2c();

    esp_err_t err = lcd_init();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "LCD init OK; running fill/border/corner/text diag");
        probe_lcd_diag();
        ESP_LOGI(TAG, "LCD diag done (border+corners+text should be on screen)");
    } else {
        ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(err));
    }

    probe_buzzer();

    /* 3. driver-level init, mirroring acq.c: QMI by identity at both
     * addresses, ST identity-probed at both with conflict detection. */
    qmi_imu_t qmi;
    lsm_imu_t st;
    bool have_qmi = qmi_imu_init(&qmi, g_i2c_bus, 0x6B) == ESP_OK;
    if (!have_qmi)
        have_qmi = qmi_imu_init(&qmi, g_i2c_bus, 0x6A) == ESP_OK;
    uint8_t qmi_addr = have_qmi ? qmi.addr : 0;
    ESP_LOGI(TAG, "QMI driver init: %s (addr 0x%02x, whoami 0x%02x)",
             have_qmi ? "OK" : "MISSING", qmi_addr,
             have_qmi ? qmi.whoami : 0);

    bool have_st = false, st_conflict = false;
    uint8_t st_addr = 0;
    const uint8_t cand[2] = { 0x6B, 0x6A };
    for (int i = 0; i < 2 && !have_st; i++) {
        if (lsm_imu_init(&st, g_i2c_bus, cand[i]) != ESP_OK)
            continue;
        if (cand[i] == qmi_addr) {
            ESP_LOGE(TAG, "ST IMU @0x%02x conflicts with live QMI address "
                     "(WHO_AM_I 0x%02x) — physical fix required: move ST "
                     "SA0 to get 0x%02x, or isolate the bus", cand[i],
                     st.whoami, cand[i] == 0x6B ? 0x6A : 0x6B);
            st_conflict = true;
            break;
        }
        have_st = true;
        st_addr = cand[i];
    }
    ESP_LOGI(TAG, "ST driver init: %s (addr 0x%02x, last whoami 0x%02x)%s",
             have_st ? "OK" : "MISSING", st_addr, st.whoami,
             st_conflict ? " CONFLICT" : "");

    /* 4. live-data loop: 15 samples, 1 Hz.  Static board -> near-constant
     * accel (|a| ~ 4096 on one axis); ask the user to tilt/wiggle during
     * the window if a movement check is wanted. */
    ESP_LOGI(TAG, "live-data loop 15 s — tilt/wiggle the board now");
    int16_t qraw[7], sraw[8];
    int qmi_fresh = 0, st_fresh = 0;
    for (int i = 0; i < 15; i++) {
        if (have_qmi) {
            bool fresh = false;
            if (qmi_imu_read(&qmi, qraw, &fresh) == ESP_OK && fresh) {
                qmi_fresh++;
                ESP_LOGI(TAG, "[%2d] QMI a=%6d %6d %6d g=%6d %6d %6d", i,
                         qraw[0], qraw[1], qraw[2],
                         qraw[3], qraw[4], qraw[5]);
            } else {
                ESP_LOGI(TAG, "[%2d] QMI read failed", i);
            }
        }
        if (have_st) {
            bool fresh = false;
            if (lsm_imu_read(&st, sraw, &fresh) == ESP_OK && fresh) {
                st_fresh++;
                ESP_LOGI(TAG, "[%2d] ST  a=%6d %6d %6d g=%6d %6d %6d", i,
                         sraw[0], sraw[1], sraw[2],
                         sraw[4], sraw[5], sraw[6]);
            } else {
                ESP_LOGI(TAG, "[%2d] ST read failed", i);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "=== VERDICT ===");
    ESP_LOGI(TAG, "QMI8658C: %s @0x%02x, fresh %d/15",
             have_qmi ? "OK" : "MISSING", qmi_addr, qmi_fresh);
    ESP_LOGI(TAG, "ST LSM6DS: %s @0x%02x (id 0x%02x), fresh %d/15%s",
             have_st ? "OK" : "MISSING", st_addr, st.whoami, st_fresh,
             st_conflict ? " ADDRESS CONFLICT" : "");
    ESP_LOGI(TAG, "=== probe done; device halts in blink loop ===");

    /* halt with a visible heartbeat so the log tail is unambiguous */
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    for (;;) {
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
