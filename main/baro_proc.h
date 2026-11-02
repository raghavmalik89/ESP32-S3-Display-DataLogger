/* baro_proc.h — raw d1/d2 -> compensated altitude for the on-board
 * estimator path. Mirrors analysis/common.py::compensate_ms5611.
 *
 * Reference pressure is FIXED at 101325 Pa (ISA).  A session-mean reference
 * (the old offline convention, mean of the first 2000 samples) is a
 * MOVING target whenever the sensor moves during that window — the 2026-10-23
 * elevator runs showed it manufacturing +0.45 m/s of fake lift for 15 s at a
 * dead-stop dwell (p0 climbing ~5 Pa/s toward the new-floor pressure while
 * 2000 samples were still accumulating).  Estimators consume velocity,
 * invariant to a constant alt offset, so a fixed reference loses nothing and
 * cannot move.  See baro_proc.c for the evidence trail.
 */
#ifndef BARO_PROC_H
#define BARO_PROC_H

#include <stdint.h>

typedef struct {
    uint16_t prom[8];
    double p0;         /* fixed ISA reference, 101325 Pa */
    int p0_ready;      /* always 1; kept for API compat */
} baro_proc_t;

void baro_proc_init(baro_proc_t *bp, const uint16_t prom[8]);

/* Compensate d1 with d2 (latest temperature sample) and return altitude in
 * metres relative to the fixed 101325 Pa reference. */
double baro_proc_altitude(baro_proc_t *bp, uint32_t d1, uint32_t d2);

/* BMP581 path: raw 24-bit on-chip-compensated pressure (Pa = raw/64) ->
 * altitude relative to the fixed 101325 Pa reference.  NaN on bad input. */
double baro_proc_altitude_bmp581(uint32_t raw_p);

int baro_proc_p0_ready(const baro_proc_t *bp);

#endif
