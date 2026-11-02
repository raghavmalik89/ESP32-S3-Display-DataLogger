/* vario_core.h — platform-independent vario estimator + audio core.
 *
 * Phase 4 WP-D. Mirrors the Python benchmark reference
 * (filter_benchmark/src/filters/stacked.py::S5_LightGateKF — the measured
 * strongest Phase-4 candidate) and the audio state machine
 * (filter_benchmark/src/audio.py) arithmetic exactly, so test-vector
 * outputs agree with the Python reference within documented tolerances.
 *
 * Portability contract:
 *   - plain C99, no heap allocation after init (all state in caller-owned
 *     structs), no libc dependency beyond what the host/MCU provides
 *   - timestamp-driven: every update takes the PHYSICAL sample timestamp;
 *     nothing assumes a fixed sample rate
 *   - deterministic: identical inputs -> identical outputs on every
 *     platform (single-threaded, no FP environment changes); FP precision
 *     is explicit: VARIO_REAL is float by default (MCU-class hardware),
 *     compile with -DVARIO_USE_DOUBLE for the reference-precision build
 *   - per-update instrumentation: callers may bracket
 *     vario_est_update() with their own cycle counter; the host runner
 *     demonstrates this.
 *
 * Integration: copy vario_core.c/.h into an ESP-IDF component or an nRF
 * Connect SDK source directory; both provide C99 compilers. No config
 * headers from the SDK are required.
 */
#ifndef VARIO_CORE_H
#define VARIO_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef VARIO_USE_DOUBLE
typedef double vario_real_t;
#else
typedef float vario_real_t;
#endif

/* ------------------------------------------------------------------ */
/* Estimator: 2-state (h, v) CV Kalman with                          */
/*   - physical plausibility SATURATION (never sample replacement —  */
/*     replacement blinds the estimator; see Phase 4 WP-A root cause),*/
/*   - innovation gating,                                             */
/*   - motion-adaptive process noise.                                 */
/* ------------------------------------------------------------------ */
typedef struct {
    vario_real_t q;            /* process noise PSD (m^2/s^3 scale) */
    vario_real_t r;            /* measurement noise variance (m^2)  */
    vario_real_t gate_sigma;   /* innovation gate, multiples of sqrt(S) */
    vario_real_t q_gain;       /* motion adaptivity gain */
    vario_real_t v_scale;      /* motion adaptivity velocity scale (m/s) */
    vario_real_t v_plaus_max;  /* plausible vertical speed bound (m/s) */
    vario_real_t plaus_margin_m;
} vario_est_config_t;

typedef struct {
    vario_real_t h, v;         /* state estimate */
    vario_real_t p00, p01, p11;/* covariance (symmetric) */
    vario_real_t q_eff;        /* adapted process noise */
    double t_prev;             /* last accepted timestamp */
    int initialized;
    unsigned long n_gated;     /* innovation-gated samples */
    unsigned long n_rejected;  /* plausibility-saturated samples */
} vario_est_state_t;

void vario_est_init(vario_est_state_t *st, const vario_est_config_t *cfg);

/* One baro altitude sample [m] at physical time t [s].
 * Returns the current vertical-speed estimate [m/s]. */
vario_real_t vario_est_update(vario_est_state_t *st,
                              const vario_est_config_t *cfg,
                              vario_real_t z_m, double t_s);

/* ------------------------------------------------------------------ */
/* Audio: two-state hysteresis latch (OFF/ON) with optional hold,    */
/* min-on, max-continuous-ON safety valve and fault veto.            */
/* ------------------------------------------------------------------ */
typedef struct {
    vario_real_t lift_on;      /* latch ON at v >= lift_on (m/s) */
    vario_real_t lift_off;     /* latch OFF at v < lift_off (m/s) */
    vario_real_t hold_s;       /* require lift_off for this long */
    vario_real_t min_on_s;     /* minimum ON episode length */
    vario_real_t max_on_s;     /* safety valve: force OFF after this (0=off) */
} vario_audio_config_t;

typedef struct {
    int state;                 /* 0 = off, 1 = on */
    double on_since;
    double off_pending;        /* < 0: none */
    unsigned long n_on, n_off, n_fault_veto, n_max_on;
} vario_audio_state_t;

void vario_audio_init(vario_audio_state_t *st);

/* One estimate sample; fault != 0 forces OFF and blocks latching. */
int vario_audio_update(vario_audio_state_t *st, const vario_audio_config_t *cfg,
                       vario_real_t v, double t_s, int fault);

/* ------------------------------------------------------------------ */
/* Generic fault detector (estimator-agnostic): streak of raw        */
/* samples deviating from the estimator's own prediction beyond a    */
/* physical bound trips the flag; a clean streak clears it.          */
/* ------------------------------------------------------------------ */
typedef struct {
    vario_real_t v_plaus_max;  /* plausible vertical speed bound (m/s) */
    vario_real_t margin_m;
    int trip_n;                /* consecutive bad samples to trip */
    int clear_n;               /* consecutive good samples to clear */
    int bad, good, flag;
} vario_fault_t;

void vario_fault_init(vario_fault_t *f, vario_real_t v_plaus_max,
                      vario_real_t margin_m, int trip_n, int clear_n);

/* z: raw altitude [m]; h_pred: estimator prediction for THIS stamp;
 * dt: seconds since the previous sample (> 0). */
int vario_fault_update(vario_fault_t *f, vario_real_t z, vario_real_t h_pred,
                       vario_real_t dt);

#ifdef __cplusplus
}
#endif
#endif /* VARIO_CORE_H */
