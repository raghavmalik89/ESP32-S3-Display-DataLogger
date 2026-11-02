/* baro_proc.c */
#include "baro_proc.h"

#include <math.h>
#include <string.h>

void baro_proc_init(baro_proc_t *bp, const uint16_t prom[8])
{
    memset(bp, 0, sizeof(*bp));
    memcpy(bp->prom, prom, sizeof(bp->prom));
    bp->p0 = 101325.0;   /* fixed ISA reference — see baro_proc_altitude */
    bp->p0_ready = 1;
}

/* exact transcription of analysis/common.py::compensate_ms5611 */
static double compensate_pa(const baro_proc_t *bp, uint32_t d1, uint32_t d2)
{
    const double c1 = bp->prom[1], c2 = bp->prom[2], c3 = bp->prom[3];
    const double c4 = bp->prom[4], c5 = bp->prom[5], c6 = bp->prom[6];
    double dt = (double)d2 - c5 * 256.0;
    double temp = 2000.0 + dt * c6 / 8388608.0;              /* 2^23 */
    double off = c2 * 65536.0 + (c4 * dt) / 128.0;           /* 2^16, 2^7 */
    double sens = c1 * 32768.0 + (c3 * dt) / 256.0;          /* 2^15, 2^8 */
    double t2 = 0.0, off2 = 0.0, sens2 = 0.0;
    if (temp < 2000.0) {                                     /* 2nd order */
        t2 = dt * dt / 2147483648.0;                         /* 2^31 */
        off2 = 5.0 * (temp - 2000.0) * (temp - 2000.0) / 2.0;
        sens2 = off2 / 2.0;
        if (temp < -1500.0) {
            off2 += 7.0 * (temp + 1500.0) * (temp + 1500.0);
            sens2 += 11.0 * (temp + 1500.0) * (temp + 1500.0) / 2.0;
        }
    }
    temp -= t2;
    off -= off2;
    sens -= sens2;
    double p01mbar = ((double)d1 * sens / 2097152.0 - off) / 32768.0;
    /* DEF-011 FIXED (2026-10-24, sensor-integration build): the ×10.0 here
     * made post-p0fix firmware velocities ×1.5498 and altitude garbage
     * (−23 725 m).  Exact transcription of analysis/common.py:
     * p * 10.0 / 1000.0 * 100.0 == p (0.01 mbar == 1 Pa). */
    return p01mbar * 10.0 / 1000.0 * 100.0;                  /* Pa */
}

/* BMP581: on-chip compensated fixed point (see bmp581.h).  Every pressure
 * sample carries its own fresh temperature — no d2 seeding, no DEF-010
 * boot-step. */
double baro_proc_altitude_bmp581(uint32_t raw_p)
{
    double p = (double)raw_p / 64.0;                           /* Pa */
    if (!(p > 0.0) || !isfinite(p)) return NAN;
    return 44330.0 * (1.0 - pow(p / 101325.0, 0.1903));
}

double baro_proc_altitude(baro_proc_t *bp, uint32_t d1, uint32_t d2)
{
    double p = compensate_pa(bp, d1, d2);
    /* reject non-finite / non-positive pressure (e.g. d2 not yet seeded at
     * boot -> compensation explodes to negative Pa) WITHOUT touching the
     * p0 accumulator: one NaN sample would poison every later altitude.
     * Callers treat NaN return as "skip this sample". */
    if (!(p > 0.0) || !isfinite(p)) return NAN;
    /* FIXED reference pressure — never a running mean.  A p0 accumulated
     * over the first N samples is a MOVING target whenever the sensor moves
     * during that window (elevator test, 2026-10-23: descending into the
     * first-2000-sample window made p0 climb ~5 Pa/s toward the lower-floor
     * pressure, producing fake +0.45 m/s lift for 15 s at a dead-stop dwell,
     * reproduced exactly by simulation, bias -0.002 m/s vs firmware tones).
     * Estimators only consume velocity, which is invariant to a constant
     * alt offset, so the ISA sea-level reference is always correct here.
     * The p0_buf accumulator below is retained but the reference is frozen
     * at 101325 Pa; p0_ready is set once at init. */
    return 44330.0 * (1.0 - pow(p / bp->p0, 0.1903));
}

int baro_proc_p0_ready(const baro_proc_t *bp) { return bp->p0_ready; }
