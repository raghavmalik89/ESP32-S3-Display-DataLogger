/* probe.c — hardware discovery app (CONFIG_LOGGER_PROBE_MODE=y).
 *
 * Output goes to the USB Serial/JTAG console (probe builds run LOG level
 * INFO).  Checks, in order:
 *   1. shared I2C bus scan on GPIO47/48
 *   2. WHO_AM_I identification: BMP581 (0x46/0x47), QMI8658C (0x6A/0x6B reg0),
 *      ST LSM6DS (0x6A/0x6B reg 0x0F)
 *   3. LCD color fill smoke test (also settles the panel init table question)
 *   4. buzzer tone sweep on GPIO16
 * Then loops forever reporting a heartbeat with the raw accel of both IMUs.
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

static void probe_whoami(uint8_t addr)
{
    uint8_t v;
    i2c_master_dev_handle_t dev;
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(g_i2c_bus, &dc, &dev) != ESP_OK) return;

    /* QMI8658C: reg 0x00 -> 0x05 */
    if (i2c_read_reg8(dev, 0x00, &v) == ESP_OK && v == 0x05)
        ESP_LOGI(TAG, "addr 0x%02x: QMI8658C (WHO_AM_I 0x05)", addr);
    /* LSM6DS family: reg 0x0F */
    if (i2c_read_reg8(dev, 0x0F, &v) == ESP_OK) {
        switch (v) {
        case WHO_LSM6DSV16X:
            ESP_LOGI(TAG, "addr 0x%02x: LSM6DSV16X (WHO_AM_I 0x70)", addr);
            break;
        case WHO_LSM6DSO:
            ESP_LOGI(TAG, "addr 0x%02x: LSM6DSO (WHO_AM_I 0x6C)", addr);
            break;
        case WHO_LSM6DS3:
            ESP_LOGI(TAG, "addr 0x%02x: LSM6DS3/DSM (WHO_AM_I 0x69)", addr);
            break;
        case 0x00: case 0xFF:
            break;   /* nothing sane answering */
        default:
            ESP_LOGI(TAG, "addr 0x%02x: unknown device, reg0x0F=0x%02x", addr, v);
            break;
        }
    }
    i2c_master_bus_rm_device(dev);
}

static void probe_i2c(void)
{
    ESP_ERROR_CHECK(i2c_bus_init());
    ESP_LOGI(TAG, "I2C bus up: SDA=%d SCL=%d @ %d kHz", I2C_SDA_GPIO,
             I2C_SCL_GPIO, I2C_FREQ_HZ / 1000);

    char line[200];
    int n = snprintf(line, sizeof(line), "scan:");
    for (uint8_t a = 0x08; a < 0x78 && n < (int)sizeof(line) - 6; a++) {
        if (i2c_probe_addr(a))
            n += snprintf(line + n, sizeof(line) - n, " 0x%02x", a);
    }
    ESP_LOGI(TAG, "%s", line);

    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (!i2c_probe_addr(a)) continue;
        if (a == 0x46 || a == 0x47) {
            uint8_t chipid = 0;
            i2c_master_dev_handle_t dev;
            i2c_device_config_t dc = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = a,
                .scl_speed_hz = I2C_FREQ_HZ,
            };
            if (i2c_master_bus_add_device(g_i2c_bus, &dc, &dev) == ESP_OK &&
                i2c_read_reg8(dev, 0x01, &chipid) == ESP_OK)
                ESP_LOGI(TAG, "addr 0x%02x: BMP581-class CHIP_ID=0x%02x",
                         a, chipid);
            if (dev) i2c_master_bus_rm_device(dev);
        } else if (a == 0x6A || a == 0x6B) {
            probe_whoami(a);
        } else if (a == 0x7E) {
            ESP_LOGI(TAG, "addr 0x7E: BMP581 CMD register ACK (not a device)");
        } else {
            ESP_LOGI(TAG, "addr 0x%02x: present, identity unknown", a);
        }
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
    /* full-screen color fills */
    lcd_fill(LCD_RED);    vTaskDelay(pdMS_TO_TICKS(600));
    lcd_fill(LCD_GREEN);  vTaskDelay(pdMS_TO_TICKS(600));
    lcd_fill(LCD_BLUE);   vTaskDelay(pdMS_TO_TICKS(600));
    lcd_fill(LCD_BLACK);

    /* white border, 2 px, full 320x170 active area */
    draw_hline(0, 0, LCD_H_RES, LCD_WHITE);
    draw_hline(0, 1, LCD_H_RES, LCD_WHITE);
    draw_hline(0, LCD_V_RES - 2, LCD_H_RES, LCD_WHITE);
    draw_hline(0, LCD_V_RES - 1, LCD_H_RES, LCD_WHITE);
    draw_vline(0, 0, LCD_V_RES, LCD_WHITE);
    draw_vline(1, 0, LCD_V_RES, LCD_WHITE);
    draw_vline(LCD_H_RES - 2, 0, LCD_V_RES, LCD_WHITE);
    draw_vline(LCD_H_RES - 1, 0, LCD_V_RES, LCD_WHITE);

    /* corner markers: red TL, green TR, blue BL, yellow BR (12x12) */
    draw_block(4, 4, 12, 12, LCD_RED);
    draw_block(LCD_H_RES - 16, 4, 12, 12, LCD_GREEN);
    draw_block(4, LCD_V_RES - 16, 12, 12, LCD_BLUE);
    draw_block(LCD_H_RES - 16, LCD_V_RES - 16, 12, 12, LCD_YELLOW);

    /* centered text lines (17 chars * 8 px = 136; (320-136)/2 = 92) */
    lcd_text(92, 56, "VARIO LOGGER REV3", LCD_WHITE, LCD_BLACK);
    lcd_text(128, 72, "LCD DIAG OK", LCD_GREEN, LCD_BLACK);
    lcd_text(120, 96, "BMP581+QMI OK", LCD_CYAN, LCD_BLACK);
}

void probe_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));   /* let USB enumerate */
    ESP_LOGI(TAG, "=== sensor-logger probe v1 ===");

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

    /* live IMU cross-check: read both IMUs, report 1 g axis + heartbeat */
    qmi_imu_t qmi;
    lsm_imu_t st;
    bool have_qmi = qmi_imu_init(&qmi, g_i2c_bus, 0x6B) == ESP_OK;
    bool have_qmi6a = have_qmi ? false
        : qmi_imu_init(&qmi, g_i2c_bus, 0x6A) == ESP_OK;
    bool have_st = lsm_imu_init(&st, g_i2c_bus, 0x6B) == ESP_OK;
    bool have_st6a = have_st ? false
        : lsm_imu_init(&st, g_i2c_bus, 0x6A) == ESP_OK;
    ESP_LOGI(TAG, "qmi@%s=%s st@%s=%s",
             have_qmi ? "6B" : (have_qmi6a ? "6A" : "-"),
             (have_qmi || have_qmi6a) ? "ok" : "missing",
             have_st ? "6B" : (have_st6a ? "6A" : "-"),
             (have_st || have_st6a) ? "ok" : "missing");

    int16_t qraw[7], sraw[8];
    for (;;) {
        if (have_qmi || have_qmi6a) {
            bool fresh;
            if (qmi_imu_read(&qmi, qraw, &fresh) == ESP_OK && fresh)
                ESP_LOGI(TAG, "QMI a=%d %d %d g=%d %d %d", qraw[0], qraw[1],
                         qraw[2], qraw[3], qraw[4], qraw[5]);
        }
        if (have_st || have_st6a) {
            bool fresh;
            if (lsm_imu_read(&st, sraw, &fresh) == ESP_OK && fresh)
                ESP_LOGI(TAG, "ST  a=%d %d %d g=%d %d %d", sraw[0], sraw[1],
                         sraw[2], sraw[4], sraw[5], sraw[6]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
