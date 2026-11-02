/* imu_lsm6dsv16x.h — LSM6DSV16X IMU via ST's official platform-independent
 * driver (lsm6dsv16x_reg.{c,h}, copied unmodified from
 * STMicroelectronics/lsm6dsv16x-pid).  This wrapper only adapts the
 * platform read/write/delay callbacks to our esp_driver_i2c master bus.
 *
 * Drop-in replacement for the MPU6050 call shape used by main.c:
 *   init -> probe WHO_AM_I (expect 0x70), BDU on, +-8 g / +-500 dps,
 *           240 Hz accel + gyro (nearest available to the 200 Hz target;
 *           120 Hz empirically NACKs more on this rig — see .c)
 *   read -> raw[0..2] accel, raw[3] temp, raw[4..6] gyro (int16)
 *           *fresh = both data-ready flags set since the previous read
 *
 * Raw scale matches the MPU +-8 g / +-500 dps convention exactly:
 * 4096 LSB/g (0.244 mg/LSB) and 65.536 LSB/dps — the existing sidecar
 * a_lsb_per_g / g_lsb_per_dps metadata values stay valid.
 */
#ifndef IMU_LSM6DSV16X_H
#define IMU_LSM6DSV16X_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "lsm6dsv16x_reg.h"

typedef struct {
    i2c_master_dev_handle_t dev;
    i2c_master_bus_handle_t bus;   /* kept for runtime bus-reset recovery */
    stmdev_ctx_t ctx;
    uint8_t addr;
    uint8_t whoami;
    uint8_t ctrl1, ctrl2, ctrl3, ctrl6, ctrl7;  /* latched readback for #C meta */
    uint8_t last_err;   /* 1 = status read failed, 2 = output burst failed,
                           3 = link recovery performed (bus reset + reconfig) */
    uint32_t recoveries;   /* runtime recovery attempts (DEF-020 diagnostic) */
    uint16_t consec_fail;  /* consecutive failed polls in OK state */
    uint16_t degraded_cnt; /* ticks since last degraded probe */
    uint16_t recover_in;   /* degraded probes until next recovery attempt */
    bool degraded;         /* link declared wedged, sparse probing */
    bool ok;
} lsm_imu_t;

/* E-frame codes emitted by main.c from last_err:
 *   1 = STATUS/drdy read failed (3 attempts)
 *   2 = output burst read failed (3 attempts)
 *   3 = IMU link recovery performed: ~200 ms of solid failure triggered
 *       degraded mode (sparse probing) and a bus reset + full reconfigure */

esp_err_t lsm_imu_init(lsm_imu_t *im, i2c_master_bus_handle_t bus, uint8_t addr);
esp_err_t lsm_imu_read(lsm_imu_t *im, int16_t out[8], bool *fresh);

#endif
