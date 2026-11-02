/* bmp581.c — see bmp581.h for the register map and data contracts. */
#include "bmp581.h"
#include "config.h"

#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMP581_REG_CHIP_ID    0x01
#define BMP581_REG_REV_ID     0x02
#define BMP581_REG_CHIP_STAT  0x11
#define BMP581_REG_DSP_IIR    0x31
#define BMP581_REG_OSR_CONFIG 0x36
#define BMP581_REG_ODR_CONFIG 0x37
#define BMP581_REG_STATUS     0x28
#define BMP581_REG_DATA       0x1D   /* TEMP XLSB..MSB then PRESS XLSB..MSB */
#define BMP581_REG_CMD        0x7E

#define BMP581_CMD_SOFT_RESET 0xB6

#define BMP581_CHIP_ID_0 0x50   /* BMP580 / BMP581 */
#define BMP581_CHIP_ID_1 0x51   /* BMP585 */

static esp_err_t wr(const bmp581_t *b, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(b->dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t rd(const bmp581_t *b, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(b->dev, &reg, 1, val, 1,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t bmp581_init(bmp581_t *b, i2c_master_bus_handle_t bus, uint8_t addr)
{
    memset(b, 0, sizeof(*b));
    b->addr = addr;
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev, &b->dev);
    if (err != ESP_OK) return err;

    uint8_t v = 0;
    err = rd(b, BMP581_REG_CHIP_ID, &v);
    if (err != ESP_OK) return err;
    b->chip_id = v;
    if (v != BMP581_CHIP_ID_0 && v != BMP581_CHIP_ID_1)
        return ESP_ERR_INVALID_RESPONSE;   /* not a BMP5 part at this addr */
    rd(b, BMP581_REG_REV_ID, &b->rev_id);

    /* soft reset, then wait for POR/softreset-complete (chip_status bit4) */
    uint8_t cmd = BMP581_CMD_SOFT_RESET;
    err = i2c_master_transmit(b->dev, (uint8_t[]){BMP581_REG_CMD, cmd}, 2,
                            pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(5));   /* reset needs ~2 ms */
    for (int i = 0; i < 20; i++) {
        err = rd(b, BMP581_REG_CHIP_STAT, &v);
        if (err == ESP_OK && (v & 0x10)) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    /* IIR bypass (offline pipeline does its own filtering) */
    err = wr(b, BMP581_REG_DSP_IIR, 0x00);
    if (err != ESP_OK) return err;

    b->ok = true;
    return ESP_OK;
}

esp_err_t bmp581_start(bmp581_t *b)
{
    if (!b->ok) return ESP_ERR_INVALID_STATE;
    /* OSR_CONFIG: press_en | press_osr<<3 | temp_osr */
    esp_err_t err = wr(b, BMP581_REG_OSR_CONFIG,
                       (uint8_t)(0x40 | (BMP581_OSR_P << 3) | BMP581_OSR_T));
    if (err != ESP_OK) return err;
    /* ODR_CONFIG: deep_dis | odr<<2 | mode(NORMAL=1) */
    return wr(b, BMP581_REG_ODR_CONFIG,
              (uint8_t)(0x80 | (BMP581_ODR << 2) | 0x01));
}

esp_err_t bmp581_status(const bmp581_t *b, uint8_t *status)
{
    return rd(b, BMP581_REG_STATUS, status);
}

esp_err_t bmp581_read_config(const bmp581_t *b, uint8_t *osr_cfg, uint8_t *odr_cfg)
{
    esp_err_t err = rd(b, BMP581_REG_OSR_CONFIG, osr_cfg);
    if (err != ESP_OK) return err;
    return rd(b, BMP581_REG_ODR_CONFIG, odr_cfg);
}

esp_err_t bmp581_read_raw(const bmp581_t *b, uint32_t *raw_t, uint32_t *raw_p)
{
    uint8_t reg = BMP581_REG_DATA;
    uint8_t d[6];
    esp_err_t err = i2c_master_transmit_receive(
        b->dev, &reg, 1, d, 6, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;
    *raw_t = ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0];
    *raw_p = ((uint32_t)d[5] << 16) | ((uint32_t)d[4] << 8) | d[3];
    return ESP_OK;
}
