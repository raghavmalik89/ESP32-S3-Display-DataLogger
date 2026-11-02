/* qmi8658c.h — compact QMI8658C driver (onboard IMU).
 *
 * Register map per QST/QMI8658 + Waveshare demo driver.  WHO_AM_I (reg 0x00)
 * reads 0x05.  Data: 12-byte burst at 0x35 (Ax..Gz, little-endian int16).
 * Scales: +-8 g -> 4096 LSB/g ; gyro range-dependent LSB/dps table below.
 *
 * Unlike the Waveshare demo this driver stays on the raw-count domain and
 * keeps the LSB/scale factors in the handle, mirroring the legacy
 * LSM6DSV16X wrapper convention (same +-8 g / +-500 dps configuration).
 */
#ifndef QMI8658C_H
#define QMI8658C_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"

#define QMI8658_ADDR_L      0x6A
#define QMI8658_ADDR_H      0x6B
#define QMI8658_WHOAMI      0x05

typedef struct {
    i2c_master_dev_handle_t dev;
    i2c_master_bus_handle_t bus;  /* kept for bus-reset recovery */
    uint8_t addr;
    uint8_t whoami;
    uint16_t g_lsb_per_dps;       /* 131/65.5/... for 250/500/... dps */
    bool ok;
} qmi_imu_t;

esp_err_t qmi_imu_init(qmi_imu_t *q, i2c_master_bus_handle_t bus, uint8_t addr);
/* +-8 g, +-500 dps, 240 Hz ODR both.  out[0..2]=accel, out[3..5]=gyro,
 * out[6]=temp raw (informational), int16. *fresh: both drdy bits set. */
esp_err_t qmi_imu_read(qmi_imu_t *q, int16_t out[7], bool *fresh);

#endif
