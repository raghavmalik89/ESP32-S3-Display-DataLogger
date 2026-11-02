/* qmi8658c.c — compact QMI8658C driver; see header for register provenance. */
#include "qmi8658c.h"
#include "config.h"
#include "i2c_bus.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* register map (subset) — addresses from the Waveshare ESP-IDF demo
 * (02_I2C_QMI8658), proven on this exact board.  Note: STATUS0 is 0x2E;
 * 0x2F is STATUS1 (CMD_DONE/WAKEUP bits) — polling 0x2F never sees DRDY. */
#define R_WHOAMI   0x00
#define R_CTRL1    0x02
#define R_CTRL2    0x03
#define R_CTRL3    0x04
#define R_CTRL5    0x06
#define R_CTRL7    0x08
#define R_CTRL8    0x0A
#define R_CTRL9    0x09
#define R_RESET    0x60
#define R_STATUS0  0x2E   /* bit0 DRDY_A, bit1 DRDY_G */
#define R_AXL      0x35   /* Ax_L .. Gz_H: 12 consecutive bytes */

/* CTRL7: accel_en | gyro_en */
#define CTRL7_AEN  0x01
#define CTRL7_GEN  0x02

static esp_err_t wr(qmi_imu_t *q, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(q->dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t rd(qmi_imu_t *q, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(q->dev, &reg, 1, buf, len,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t qmi_imu_init(qmi_imu_t *q, i2c_master_bus_handle_t bus, uint8_t addr)
{
    memset(q, 0, sizeof(*q));
    q->bus = bus;
    q->addr = addr;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &q->dev);
    if (err != ESP_OK) return err;

    err = i2c_read_reg8(q->dev, R_WHOAMI, &q->whoami);
    if (err != ESP_OK || q->whoami != QMI8658_WHOAMI) {
        q->ok = false;
        return ESP_ERR_NOT_FOUND;
    }

    /* Soft reset + on-demand calibration, exactly as the Waveshare demo
     * does: the factory firmware leaves the part in an unknown mode
     * (FIFO etc.) and without the reset DRDY never asserts.  Cost: ~2.4 s
     * of boot time, once. */
    wr(q, R_RESET, 0xB0);
    vTaskDelay(pdMS_TO_TICKS(50));
    wr(q, R_CTRL9, 0xA2);            /* on-demand cali */
    vTaskDelay(pdMS_TO_TICKS(2200));
    wr(q, R_CTRL9, 0x00);            /* NOP */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* CTRL8 = 0xc0 and CTRL1 = 0x60 per the demo init (data-ready pin
     * config + int push-pull); no interrupts are wired, we poll STATUS0. */
    wr(q, R_CTRL8, 0xC0);
    wr(q, R_CTRL1, 0x60);
    /* CTRL2: accel +-8 g (0x2<<4), ODR 250 Hz code 0x05 */
    wr(q, R_CTRL2, 0x20 | 0x05);
    /* CTRL3: gyro +-500 dps (0x5<<4), ODR 250 Hz */
    wr(q, R_CTRL3, 0x50 | 0x05);
    /* CTRL5: read-modify-write, LPF disabled (demo pattern; preserves
     * the high nibble reserved bits instead of clobbering them) */
    uint8_t c5 = 0;
    rd(q, R_CTRL5, &c5, 1);
    wr(q, R_CTRL5, c5 & (uint8_t)~0x11);
    /* CTRL7: enable accel + gyro */
    wr(q, R_CTRL7, CTRL7_AEN | CTRL7_GEN);

    q->g_lsb_per_dps = 65;        /* 500 dps full scale -> 65.5 LSB/dps */
    vTaskDelay(pdMS_TO_TICKS(10));
    q->ok = true;
    return ESP_OK;
}

esp_err_t qmi_imu_read(qmi_imu_t *q, int16_t out[7], bool *fresh)
{
    uint8_t st = 0;
    esp_err_t err = rd(q, R_STATUS0, &st, 1);
    if (err != ESP_OK) return err;
    *fresh = (st & 0x03) == 0x03;

    uint8_t buf[12];
    err = rd(q, R_AXL, buf, 12);
    if (err != ESP_OK) return err;
    for (int i = 0; i < 6; i++)
        out[i] = (int16_t)((uint16_t)buf[2 * i + 1] << 8 | buf[2 * i]);
    out[6] = 0;   /* temp not used in the fast path */
    return ESP_OK;
}
