/* i2c_bus.h — shared I2C master bus + recovery helpers. */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

extern i2c_master_bus_handle_t g_i2c_bus;

esp_err_t i2c_bus_init(void);

/* 1-byte register read (common WHO_AM_I pattern). Returns ESP_OK + value. */
esp_err_t i2c_read_reg8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val);

/* Probe an address without creating a device handle. */
bool i2c_probe_addr(uint8_t addr);

/* Count of bus-reset recoveries performed (meta/reporting). */
uint32_t i2c_bus_recovery_count(void);

#endif
