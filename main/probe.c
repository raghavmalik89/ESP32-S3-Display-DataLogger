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
#include "esp_rom_sys.h"
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
    esp_err_t err;
    i2c_master_dev_handle_t dev;
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(g_i2c_bus, &dc, &dev) != ESP_OK) return;

    /* QMI8658C: reg 0x00 -> 0x05 */
    v = 0xAA;
    err = i2c_read_reg8(dev, 0x00, &v);
    ESP_LOGI(TAG, "addr 0x%02x reg0x00: err=%d val=0x%02x", addr, err, v);
    /* read again after 100 ms in case the part needs wake-up time */
    vTaskDelay(pdMS_TO_TICKS(100));
    v = 0xAA;
    err = i2c_read_reg8(dev, 0x00, &v);
    ESP_LOGI(TAG, "addr 0x%02x reg0x00 (retry): err=%d val=0x%02x", addr,
             err, v);
    /* hypothesis: QMI8658 powers up with its register block held in reset;
     * issue the soft-reset command, wait, then read WHO_AM_I again */
    {
        uint8_t rst[2] = { 0x60, 0xB0 };
        esp_err_t werr = i2c_master_transmit(dev, rst, 2,
                                             pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        vTaskDelay(pdMS_TO_TICKS(60));
        v = 0xAA;
        err = i2c_read_reg8(dev, 0x00, &v);
        ESP_LOGI(TAG, "addr 0x%02x reg0x00 (after reset, werr=%d): err=%d "
                 "val=0x%02x", addr, werr, err, v);
        /* register dump 0x00..0x0F + STATUS0 — is ANY register non-zero? */
        char dump[80];
        int n = snprintf(dump, sizeof(dump), "regs:");
        for (uint8_t reg = 0x00; reg <= 0x0F; reg++) {
            uint8_t rv = 0xEE;
            if (i2c_read_reg8(dev, reg, &rv) != ESP_OK) rv = 0xFF;
            n += snprintf(dump + n, sizeof(dump) - n, " %02x=%02x", reg, rv);
        }
        ESP_LOGI(TAG, "addr 0x%02x %s", addr, dump);
        /* cali command then status + whoami */
        uint8_t cali[2] = { 0x09, 0xA2 };
        i2c_master_transmit(dev, cali, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        vTaskDelay(pdMS_TO_TICKS(2500));
        uint8_t s0 = 0xEE;
        i2c_read_reg8(dev, 0x2E, &s0);
        v = 0xAA;
        err = i2c_read_reg8(dev, 0x00, &v);
        ESP_LOGI(TAG, "addr 0x%02x after cali: status0=0x%02x whoami err=%d "
                 "val=0x%02x", addr, s0, err, v);
        /* two-phase transaction (STOP then START) instead of repeated start */
        {
            uint8_t reg0 = 0x00;
            uint8_t twophase = 0xAA;
            esp_err_t e1 = i2c_master_transmit(dev, &reg0, 1,
                                               pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            esp_err_t e2 = i2c_master_receive(dev, &twophase, 1,
                                              pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            ESP_LOGI(TAG, "addr 0x%02x two-phase: e1=%d e2=%d val=0x%02x",
                     addr, e1, e2, twophase);
            /* hypothesis: CTRL1 bit4 "sensor enable" gates ID readback.
             * Write CTRL1 = 0x60 (per known-good init sequences), then
             * re-read WHO_AM_I (0x00) and REVISION_ID (0x01). */
            uint8_t en[2] = { 0x02, 0x60 };
            esp_err_t e3 = i2c_master_transmit(dev, en, 2,
                                               pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            vTaskDelay(pdMS_TO_TICKS(10));
            uint8_t who = 0xAA, rev = 0xAA, ctrl1 = 0xAA;
            esp_err_t e4 = i2c_read_reg8(dev, 0x00, &who);
            i2c_read_reg8(dev, 0x01, &rev);
            esp_err_t e5 = i2c_read_reg8(dev, 0x02, &ctrl1);
            ESP_LOGI(TAG, "addr 0x%02x after CTRL1=0x60: werr=%d rd err=%d/%d "
                     "WHO=0x%02x REV=0x%02x CTRL1rd=0x%02x", addr, e3, e4, e5,
                     who, rev, ctrl1);
            /* also try 0x40 variant */
            uint8_t en2[2] = { 0x02, 0x40 };
            i2c_master_transmit(dev, en2, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            vTaskDelay(pdMS_TO_TICKS(10));
            who = 0xAA; rev = 0xAA;
            i2c_read_reg8(dev, 0x00, &who);
            i2c_read_reg8(dev, 0x01, &rev);
            ESP_LOGI(TAG, "addr 0x%02x after CTRL1=0x40: WHO=0x%02x REV=0x%02x",
                     addr, who, rev);
            /* functional test: full accel+gyro enable, then read STATUS0
             * (0x2E) and the 12 data bytes at 0x35 twice, 500 ms apart.
             * If the chip is alive but only the ID readback is odd, the
             * data registers will show DRDY and changing values. */
            uint8_t full[8][2] = {
                { 0x02, 0x60 },  /* CTRL1: I2C + enable */
                { 0x03, 0x00 },  /* CTRL2: accel off (set below) */
                { 0x04, 0x00 },  /* CTRL3: gyro off */
                { 0x05, 0x11 },  /* CTRL4: no lpf */
                { 0x06, 0x00 },  /* CTRL5 */
                { 0x08, 0x03 },  /* CTRL7: enable accel+gyro */
            };
            for (int k = 0; k < 6; k++)
                i2c_master_transmit(dev, full[k], 2,
                                    pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            /* accel 200 Hz, 16g: CTRL2 = 0x2A; gyro 2000dps: CTRL3 = 0x36 */
            uint8_t a2[2] = { 0x03, 0x2A };
            uint8_t g2[2] = { 0x04, 0x36 };
            i2c_master_transmit(dev, a2, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            i2c_master_transmit(dev, g2, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            vTaskDelay(pdMS_TO_TICKS(100));
            for (int pass = 0; pass < 2; pass++) {
                uint8_t s0 = 0xEE;
                i2c_read_reg8(dev, 0x2E, &s0);
                uint8_t d[12];
                memset(d, 0xEE, sizeof(d));
                uint8_t dreg = 0x35;
                esp_err_t de = i2c_master_transmit_receive(
                    dev, &dreg, 1, d, 12, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
                ESP_LOGI(TAG, "addr 0x%02x functional pass %d: status0=0x%02x "
                         "derr=%d data=%02x%02x %02x%02x %02x%02x %02x%02x "
                         "%02x%02x %02x%02x", addr, pass, s0, de,
                         d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
                         d[8], d[9], d[10], d[11]);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
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

/* Waveshare demo (working reference) reads WHO_AM_I=0x05 directly on
 * GPIO47/48 @ 300 kHz.  Ours uses the new i2c_master driver @ 100 kHz
 * and gets 0x00 (IDF 6.1 removed the legacy driver, so same new-driver
 * transport at both speeds).  Isolates bus-speed vs dead-silicon. */
static void legacy_i2c_whoami(void)
{
    const uint32_t speeds[2] = { 300 * 1000, 100 * 1000 };
    for (int s = 0; s < 2; s++) {
        i2c_master_bus_config_t bc = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_NUM_0,
            .sda_io_num = 47,
            .scl_io_num = 48,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        i2c_master_bus_handle_t bus;
        esp_err_t e = i2c_new_master_bus(&bc, &bus);
        i2c_device_config_t dc = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x6B,
            .scl_speed_hz = speeds[s],
        };
        i2c_master_dev_handle_t dev = NULL;
        if (e == ESP_OK) e = i2c_master_bus_add_device(bus, &dc, &dev);
        ESP_LOGI(TAG, "new-driver @%lu kHz: init err=%d",
                 (unsigned long)(speeds[s] / 1000), e);
        if (e == ESP_OK) {
            uint8_t reg = 0x00;
            for (int i = 0; i < 5; i++) {
                uint8_t v = 0xAA;
                esp_err_t re = i2c_master_transmit_receive(
                    dev, &reg, 1, &v, 1, pdMS_TO_TICKS(1000));
                ESP_LOGI(TAG, "@%luk WHO_AM_I try %d: err=%d val=0x%02x",
                         (unsigned long)(speeds[s] / 1000), i, re, v);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        if (dev) i2c_master_bus_rm_device(dev);
        if (bus) i2c_del_master_bus(bus);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Bit-banged I2C last resort: full control of START/setup/hold timing,
 * in case the ESP32-S3 hardware I2C driver's edges disagree with the
 * chip on this board (legacy Arduino driver is gone from IDF 6.1). */
#define BB_SDA 47
#define BB_SCL 48
#define BB_HALF_US 6

static void bb_release(int pin) { gpio_set_level(pin, 1); }
static bool bb_read(int pin) { return gpio_get_level(pin) != 0; }

static void bb_start(void)
{
    bb_release(BB_SDA);
    bb_release(BB_SCL);
    esp_rom_delay_us(BB_HALF_US);
    gpio_set_level(BB_SDA, 0);
    esp_rom_delay_us(BB_HALF_US);
    gpio_set_level(BB_SCL, 0);
    esp_rom_delay_us(BB_HALF_US);
}

static void bb_stop(void)
{
    gpio_set_level(BB_SDA, 0);
    esp_rom_delay_us(BB_HALF_US);
    bb_release(BB_SCL);
    esp_rom_delay_us(BB_HALF_US);
    bb_release(BB_SDA);
    esp_rom_delay_us(BB_HALF_US);
}

static bool bb_write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(BB_SDA, (b >> i) & 1);
        esp_rom_delay_us(BB_HALF_US);
        bb_release(BB_SCL);
        esp_rom_delay_us(BB_HALF_US * 2);
        gpio_set_level(BB_SCL, 0);
        esp_rom_delay_us(BB_HALF_US);
    }
    bb_release(BB_SDA);
    esp_rom_delay_us(BB_HALF_US);
    bb_release(BB_SCL);
    esp_rom_delay_us(BB_HALF_US);
    bool ack = !bb_read(BB_SDA);
    gpio_set_level(BB_SCL, 0);
    esp_rom_delay_us(BB_HALF_US);
    return ack;
}

static uint8_t bb_read_byte(bool ack)
{
    uint8_t b = 0;
    bb_release(BB_SDA);
    for (int i = 7; i >= 0; i--) {
        esp_rom_delay_us(BB_HALF_US);
        bb_release(BB_SCL);
        esp_rom_delay_us(BB_HALF_US);
        if (bb_read(BB_SDA)) b |= (uint8_t)(1 << i);
        esp_rom_delay_us(BB_HALF_US);
        gpio_set_level(BB_SCL, 0);
        esp_rom_delay_us(BB_HALF_US);
    }
    gpio_set_level(BB_SDA, !ack);
    esp_rom_delay_us(BB_HALF_US);
    bb_release(BB_SCL);
    esp_rom_delay_us(BB_HALF_US * 2);
    gpio_set_level(BB_SCL, 0);
    esp_rom_delay_us(BB_HALF_US);
    bb_release(BB_SDA);
    return b;
}

static void bitbang_whoami(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BB_SDA) | (1ULL << BB_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io);
    bb_release(BB_SDA);
    bb_release(BB_SCL);
    vTaskDelay(pdMS_TO_TICKS(10));

    for (int t = 0; t < 5; t++) {
        bb_start();
        bool a1 = bb_write_byte(0x6B << 1);         /* address + W */
        bool a2 = bb_write_byte(0x00);              /* register 0 */
        bb_start();                                 /* repeated start */
        bool a3 = bb_write_byte((0x6B << 1) | 1);   /* address + R */
        uint8_t v = bb_read_byte(false);
        bb_stop();
        ESP_LOGI(TAG, "bitbang WHO_AM_I try %d: acks=%d%d%d val=0x%02x",
                 t, a1, a2, a3, v);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /* also read REVISION_ID 0x01 and CTRL1 0x02 for reference */
    bb_start();
    bb_write_byte(0x6B << 1);
    bb_write_byte(0x01);
    bb_start();
    bb_write_byte((0x6B << 1) | 1);
    uint8_t rev = bb_read_byte(false);
    bb_stop();
    bb_start();
    bb_write_byte(0x6B << 1);
    bb_write_byte(0x02);
    bb_start();
    bb_write_byte((0x6B << 1) | 1);
    uint8_t c1 = bb_read_byte(false);
    bb_stop();
    ESP_LOGI(TAG, "bitbang REV=0x%02x CTRL1=0x%02x", rev, c1);
}

static void probe_i2c(void)
{
    legacy_i2c_whoami();
    bitbang_whoami();


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
