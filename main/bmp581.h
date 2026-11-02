/* bmp581.h — minimal BMP581 driver over the ESP-IDF i2c_master bus.
 *
 * BMP5-generation part: on-chip trim compensation.  The host does NOT read
 * trim coefficients; the 24-bit data registers are pre-compensated fixed
 * point:
 *   pressure [Pa] = raw24_p / 64.0
 *   temp    [°C] = raw24_t / 65536.0
 * (Bosch BMP5_SensorAPI: bmp5_get_sensor_data, raw_data_p / 64.0 in Pa.)
 *
 * Register map (Bosch BMP5_SensorAPI bmp5_defs.h):
 *   0x01 CHIP_ID (0x50=BMP580/581, 0x51=BMP585)   0x02 REV_ID
 *   0x11 CHIP_STATUS (bit4 POR/softreset complete)
 *   0x1D..0x1F TEMP_DATA XLSB/LSB/MSB   0x20..0x22 PRESS_DATA
 *   0x28 STATUS (bit0 drdy, bit1 nvm_rdy, bit2 nvm_err, bit3 nvm_cmd_err)
 *   0x36 OSR_CONFIG (bit6 press_en, bits5:3 press osr, bits2:0 temp osr)
 *   0x37 ODR_CONFIG (bit7 deep_dis, bits6:2 odr, bits1:0 power mode)
 *   0x31 DSP_IIR (IIR bypass = 0x00 — we do our own filtering offline)
 *   0x7E CMD (0xB6 soft reset)
 * Note: 0x7E is the command register — an I2C bus scan "finds" it because
 * the part ACKs its own CMD address.  It is NOT a second device.
 */
#ifndef BMP581_H
#define BMP581_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t addr;
    uint8_t chip_id;
    uint8_t rev_id;
    bool ok;
} bmp581_t;

esp_err_t bmp581_init(bmp581_t *b, i2c_master_bus_handle_t bus, uint8_t addr);
/* Start NORMAL-mode sampling at the configured ODR/OSR (set in config.h). */
esp_err_t bmp581_start(bmp581_t *b);
/* Read STATUS; returns true (and sets *drdy) without touching data regs. */
esp_err_t bmp581_status(const bmp581_t *b, uint8_t *status);
/* Read back OSR_CONFIG/ODR_CONFIG as actually latched in the part
 * (provenance for the analysis sidecar — proves the config write landed). */
esp_err_t bmp581_read_config(const bmp581_t *b, uint8_t *osr_cfg, uint8_t *odr_cfg);
/* Burst-read TEMP(3)+PRESS(3) at 0x1D; returns fixed-point raw24 values. */
esp_err_t bmp581_read_raw(const bmp581_t *b, uint32_t *raw_t, uint32_t *raw_p);

#endif
