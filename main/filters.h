/* filters.h — vario filter bank: C03, D01, S4, S5.
 *
 * Faithful C ports of the benchmarked Python references
 * (filter_benchmark/src/filters/{tracking,kalman,stacked}.py), frozen
 * tested configurations, timestamp-driven (physical sample time in seconds,
 * dt from consecutive timestamps, dt<=0 repeats the last estimate).
 *
 * S5 reuses the already-ported vario_core estimator (same algorithm).
 * All four run in parallel on every baro sample; the GUI/audio consumer
 * selects one output.  State is caller-owned; no heap after init.
 *
 * Config provenance (identical to the FIRMWARE_BANK benchmark):
 *   C03  CV tracker, steady-state Riccati gains (q=1.0, r=0.035)
 *   D01  full 2-state CV Kalman      (q=1.0, r=0.035)
 *   S4   fast/slow gated blend (fast C01 tau=0.35, slow D01 q=0.3 r=0.035)
 *   S5   light-gate robust CV KF     (q=0.3, r=0.035, gate 3.5, adaptQ)
 */
#ifndef FILTERS_H
#define FILTERS_H

#include <stddef.h>
#include "vario_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *id;                    /* short id shown on screen: "C03" */
    const char *name;                  /* full benchmark name */
    size_t state_size;
    void (*reset)(void *st);
    /* one altitude sample [m] at physical time t [s]; returns v [m/s] */
    float (*update)(void *st, float z_m, double t_s);
    float (*altitude)(const void *st);
} filter_def_t;

/* bank order = Vario-screen cycle order: C03 -> D01 -> S4 -> S5 */
#define FILT_C03 0
#define FILT_D01 1
#define FILT_S4  2
#define FILT_S5  3
#define FILT_N   4

extern const filter_def_t g_filters[FILT_N];

const char *filter_id(int f);

#ifdef __cplusplus
}
#endif
#endif
