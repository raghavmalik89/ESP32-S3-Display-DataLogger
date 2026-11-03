/* acq.h — timestamped sensor acquisition, filter bank, graph history.
 *
 * One task at 200 Hz reads the QMI8658C; every 5th tick (40 Hz, matching
 * the BMP581 ODR) reads the baro, feeds all four filters, updates the
 * audio latch and the graph rings.  Nothing here blocks on display, USB
 * or storage: samples are pushed to lock-free queues and dropped with a
 * counter if the consumer falls behind (sequence gaps tell the host).
 */
#ifndef ACQ_H
#define ACQ_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* pushed by acq to tx.c queues */
typedef struct {
    uint32_t seq;
    uint64_t t_us;
    uint32_t p_raw, t_raw;      /* BMP581 raw24 */
} acq_baro_evt_t;

typedef struct {
    uint32_t seq;
    uint64_t t_us;
    int16_t acc[3], gyr[3];     /* raw counts */
    uint8_t sid;                /* 0 = QMI8658C, 1 = external ST IMU */
} acq_imu_evt_t;

/* sensor discovery + driver init; call once before acq_start() */
esp_err_t acq_init(void);

/* start the acquisition task */
esp_err_t acq_start(void);

/* select the active filter (FILT_*).  Filters are never reset on switch:
 * each keeps its own converged state, so switching causes no spikes. */
void acq_select_filter(int f);

#ifdef __cplusplus
}
#endif
#endif
