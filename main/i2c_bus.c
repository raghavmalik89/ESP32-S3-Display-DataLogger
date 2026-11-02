/* i2c_bus.c */
#include "i2c_bus.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

i2c_master_bus_handle_t g_i2c_bus;
static uint32_t s_recoveries;

esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    s_recoveries = 0;
    return i2c_new_master_bus(&cfg, &g_i2c_bus);
}

bool i2c_probe_addr(uint8_t addr)
{
    return i2c_master_probe(g_i2c_bus, addr, pdMS_TO_TICKS(I2C_TIMEOUT_MS))
           == ESP_OK;
}

esp_err_t i2c_read_reg8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* Reset the bus FSM after sustained NAK/wedge. Deletes and recreates the
 * master bus; device handles are invalidated by the caller's recovery path
 * (see imu_lsm6dsv16x.c legacy DEF-020 behavior). */
esp_err_t i2c_bus_reset(void)
{
    i2c_master_bus_handle_t old = g_i2c_bus;
    esp_err_t err = i2c_del_master_bus(old);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_err_t err2 = i2c_bus_init();
    if (err == ESP_OK && err2 == ESP_OK) s_recoveries++;
    return (err == ESP_OK) ? err2 : err;
}

uint32_t i2c_bus_recovery_count(void) { return s_recoveries; }
