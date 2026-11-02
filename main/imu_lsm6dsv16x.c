/* imu_lsm6dsv16x.c — see imu_lsm6dsv16x.h for the contract. */
#include "imu_lsm6dsv16x.h"
#include "config.h"

#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---- platform callbacks for the ST driver -------------------------------- */
/* ST passes the ctx->handle we set = the i2c_master device handle. */
static int32_t io_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint8_t tmp[32];
    if (len + 1 > sizeof(tmp)) return -1;
    tmp[0] = reg;
    memcpy(tmp + 1, buf, len);
    return i2c_master_transmit((i2c_master_dev_handle_t)handle, tmp, len + 1,
                               pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK ? 0 : -1;
}

static int32_t io_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return i2c_master_transmit_receive((i2c_master_dev_handle_t)handle,
                                       &reg, 1, buf, len,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK
               ? 0 : -1;
}

static void io_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* Some clones/breakouts NACK the first transaction after an ODR switch or
 * when the internal sampler needs a moment. A short bounded retry turns a
 * ~70/s NACK storm (observed on bringup #1) into a handful of absorbed
 * hiccups.  2 ms between attempts makes the 3-attempt span ~4 ms, i.e. one
 * full 240 Hz conversion period — a read that collides with a conversion
 * has a fresh window on the next attempt.  Worst-case cost ~6 ms only on
 * already-failing ticks; bench_i2c_call/bench_baro_miss catch any blowout. */
#define ST_RETRY(expr)                                  \
    ({ int32_t _r; int _i;                              \
       for (_i = 0; _i < 3; _i++) {                     \
           _r = (expr);                                 \
           if (_r == 0) break;                          \
           vTaskDelay(pdMS_TO_TICKS(2));                \
       }                                                \
       _r; })

static esp_err_t lsm_imu_configure(lsm_imu_t *im)
{
    /* Probe with retry: after an MCU reset the sensor keeps its previous
     * configuration and its I2C front-end can NACK the first transactions
     * (same root cause as the read-path retries below).  Give it a few
     * attempts with settle delays before declaring it missing. */
    im->whoami = 0;
    for (int i = 0; i < 5; i++) {
        if (lsm6dsv16x_device_id_get(&im->ctx, &im->whoami) == 0
            && im->whoami == LSM6DSV16X_ID)
            break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (im->whoami != LSM6DSV16X_ID)
        return ESP_ERR_INVALID_RESPONSE;   /* not an LSM6DSV16X at this addr */

    if (ST_RETRY(lsm6dsv16x_block_data_update_set(&im->ctx, 1)) != 0) return ESP_FAIL;
    if (ST_RETRY(lsm6dsv16x_xl_full_scale_set(&im->ctx, LSM6DSV16X_8g)) != 0) return ESP_FAIL;
    if (ST_RETRY(lsm6dsv16x_gy_full_scale_set(&im->ctx, LSM6DSV16X_500dps)) != 0) return ESP_FAIL;
    /* ODR 240 Hz (nearest available to the 200 Hz target).  Empirically
     * beats 120 Hz on this rig: at 120 Hz the NACK rate roughly doubled
     * (905 vs 444 E-frames per 30 s, stale reuse 11.7% vs 1.9%) — the
     * simple "conversion-collision" model is wrong and the residual
     * failures look electrical/contact-related (hand-installed breakout),
     * not rate-related.  See docs/lsm6dsv16x_integration.md. */
    if (ST_RETRY(lsm6dsv16x_xl_data_rate_set(&im->ctx, LSM6DSV16X_ODR_AT_240Hz)) != 0) return ESP_FAIL;
    if (ST_RETRY(lsm6dsv16x_gy_data_rate_set(&im->ctx, LSM6DSV16X_ODR_AT_240Hz)) != 0) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(30));   /* first samples after ODR enable */
    return ESP_OK;
}

esp_err_t lsm_imu_init(lsm_imu_t *im, i2c_master_bus_handle_t bus, uint8_t addr)
{
    memset(im, 0, sizeof(*im));
    im->addr = addr;
    im->bus = bus;
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ_A,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev, &im->dev);
    if (err != ESP_OK) return err;

    im->ctx.write_reg = io_write;
    im->ctx.read_reg = io_read;
    im->ctx.mdelay = io_delay;
    im->ctx.handle = im->dev;

    err = lsm_imu_configure(im);
    if (err != ESP_OK) {
        /* MCU reset can leave the slave holding SDA (mid-transaction
         * wedge); bringup #4/#5/#7/#8 hit this ~50% of resets.  The IDF
         * recovery bit-bangs SCL to clock out the stuck byte.  If the
         * wedge is internal rather than a held bus this changes nothing
         * and we still fail -> health_fail_halt (power-cycle required). */
        i2c_master_bus_reset(bus);
        vTaskDelay(pdMS_TO_TICKS(20));
        err = lsm_imu_configure(im);
    }
    if (err != ESP_OK) return err;

    /* config readback for the #C sidecar meta (proves the writes landed) */
    uint8_t v = 0;
    if (io_read(im->dev, 0x10, &v, 1) == 0) im->ctrl1 = v;   /* CTRL1: xl odr/fs */
    if (io_read(im->dev, 0x11, &v, 1) == 0) im->ctrl2 = v;   /* CTRL2: gy odr/fs */
    if (io_read(im->dev, 0x12, &v, 1) == 0) im->ctrl3 = v;   /* CTRL3 */
    if (io_read(im->dev, 0x15, &v, 1) == 0) im->ctrl6 = v;   /* CTRL6: xl hm/ftype */
    if (io_read(im->dev, 0x16, &v, 1) == 0) im->ctrl7 = v;   /* CTRL7: gy hm */

    im->ok = true;
    return ESP_OK;
}

/* Runtime link recovery (DEF-020): when the IMU goes deaf mid-session —
 * observed as progressive NACK increase culminating in permanent deafness
 * (~25 s into a static capture) — a naive poll-and-retry every 5 ms tick
 * burns 3 attempts + 2 ms delays per tick, stretching the shared tick from
 * 5.0 to ~6.4 ms and dragging the BMP581 cadence from 25 ms to 32 ms
 * (every-5th-tick scheduling).  Instead: after 200 ms of solid failure,
 * enter DEGRADED mode (one cheap probe every 100 ms, one E frame per
 * failed probe) and every 2 s try bus-reset + full reconfigure.  Any
 * successful probe exits DEGRADED immediately. */
#define LSM_OK_CONSEC_FAIL   40    /* ticks (5 ms) of solid failure -> degrade */
#define LSM_DEGRADED_PERIOD  20    /* probe every 20th tick = 100 ms */
#define LSM_RECOVER_EVERY    20    /* re-init every 20 probes = 2 s */

static void lsm_imu_recover(lsm_imu_t *im)
{
    im->recoveries++;
    i2c_master_bus_reset(im->bus);
    vTaskDelay(pdMS_TO_TICKS(20));
    (void)lsm_imu_configure(im);   /* best effort; probe decides if it worked */
}

esp_err_t lsm_imu_read(lsm_imu_t *im, int16_t out[8], bool *fresh)
{
    uint8_t b[14];
    lsm6dsv16x_data_ready_t drdy = {0};

    if (im->degraded) {
        if (++im->degraded_cnt < LSM_DEGRADED_PERIOD) {
            *fresh = false;
            return ESP_OK;   /* cheap skip: no bus traffic, no tick stretch */
        }
        im->degraded_cnt = 0;
        if (lsm6dsv16x_flag_data_ready_get(&im->ctx, &drdy) != 0) {
            im->last_err = 1;
            if (++im->recover_in >= LSM_RECOVER_EVERY) {
                im->recover_in = 0;
                lsm_imu_recover(im);
                im->last_err = 3;   /* E3: recovery performed */
            }
            return ESP_FAIL;
        }
        /* probe succeeded: link is back */
        im->consec_fail = 0;
        im->recover_in = 0;
        im->degraded = false;
        *fresh = drdy.drdy_xl && drdy.drdy_gy;
        if (!*fresh) return ESP_OK;
        /* fall through to the burst with the drdy we already have */
        if (ST_RETRY(im->ctx.read_reg(im->ctx.handle, LSM6DSV16X_OUT_TEMP_L,
                                      b, sizeof(b))) != 0) {
            im->last_err = 2;
            return ESP_FAIL;
        }
    } else {
        if (ST_RETRY(lsm6dsv16x_flag_data_ready_get(&im->ctx, &drdy)) != 0) {
            im->last_err = 1;
            if (++im->consec_fail >= LSM_OK_CONSEC_FAIL) {
                im->degraded = true;
                im->degraded_cnt = 0;
            }
            return ESP_FAIL;
        }
        im->consec_fail = 0;
        *fresh = drdy.drdy_xl && drdy.drdy_gy;
        if (!*fresh) return ESP_OK;   /* caller reuses the previous raw[] */

        /* One atomic 14-byte burst instead of three separate ST calls:
         * OUT_TEMP_L .. OUTZ_H_A are contiguous (0x20..0x2D) and IF_INC is
         * on.  Halves the transactions per fresh sample (two collision
         * chances -> one) and removes the tear risk between the accel and
         * gyro blocks. */
        if (ST_RETRY(im->ctx.read_reg(im->ctx.handle, LSM6DSV16X_OUT_TEMP_L,
                                      b, sizeof(b))) != 0) {
            im->last_err = 2;
            return ESP_FAIL;
        }
    }

    out[3] = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    for (int i = 0; i < 3; i++)
        out[4 + i] = (int16_t)((uint16_t)b[2 + 2 * i]
                               | ((uint16_t)b[3 + 2 * i] << 8));  /* gyro */
    for (int i = 0; i < 3; i++)
        out[i] = (int16_t)((uint16_t)b[8 + 2 * i]
                           | ((uint16_t)b[9 + 2 * i] << 8));      /* accel */
    out[7] = 0;
    return ESP_OK;
}
