/* audio_sched.h — pure-C99 vario beep profile + phase scheduler.
 *
 * Host-portable by design (no ESP-IDF, no libc beyond stdbool/stdint):
 * the deterministic host test (tests/audio_sched_test.c) compiles this
 * header together with the production vario_core.c latch and exercises
 * the exact code that ships in the firmware.
 *
 * Scheduling contract (Audio Rev 2):
 *   - The beep phase (its ON/OFF timeline) is NEVER restarted by pitch
 *     or timing changes.  s_beep_t0-style resets on every sample were the
 *     Rev-1 defect: fluctuating vertical speed changed the interpolated
 *     parameters every tick and produced irregular beeping.
 *   - A fresh beep cycle starts only when entering the climb-audio
 *     state.  After that, ON/OFF boundaries advance monotonically and
 *     each new phase picks up the CURRENT interpolated parameters at
 *     its boundary ("apply at the next phase boundary").
 *   - Pitch (frequency) is updated immediately, without touching the
 *     timing cycle.
 *   - Callers pass the monotonic sample timestamp; nothing here blocks.
 */
#ifndef AUDIO_SCHED_H
#define AUDIO_SCHED_H

#include <stdbool.h>
#include <stdint.h>

/* ---- beep profile: climb m/s -> {freq Hz, ON ms, OFF ms} ---------------
 * Initial tuning values (Audio Rev 2), shaped against the published
 * XC Tracer Maxx II cadence (~552 ms period at +1.16 m/s, 332 ms at
 * +6.0 m/s).  NOT flight-validated. */
typedef struct {
    float v;            /* climb rate anchor (m/s) */
    uint16_t freq_hz;
    uint16_t on_ms;
    uint16_t off_ms;
} au_prof_t;

static const au_prof_t AU_PROFILE[] = {
    { 0.20f,  500, 250, 500 },
    { 0.50f,  520, 250, 450 },
    { 1.00f,  560, 250, 370 },
    { 2.00f,  670, 270, 290 },
    { 3.00f,  810, 270, 230 },
    { 5.00f, 1100, 230, 170 },
    { 7.00f, 1375, 180, 120 },
};
#define AU_PROFILE_N (sizeof(AU_PROFILE) / sizeof(AU_PROFILE[0]))

/* Interpolated (clamped outside the table) profile for climb v > 0. */
static inline void au_profile(float v, int *hz, int *on_ms, int *off_ms)
{
    const au_prof_t *a = &AU_PROFILE[0];
    const au_prof_t *b = &AU_PROFILE[AU_PROFILE_N - 1];
    if (v <= a->v) {
        *hz = a->freq_hz; *on_ms = a->on_ms; *off_ms = a->off_ms;
        return;
    }
    if (v >= b->v) {
        *hz = b->freq_hz; *on_ms = b->on_ms; *off_ms = b->off_ms;
        return;
    }
    for (unsigned i = 0; i + 1 < AU_PROFILE_N; i++) {
        a = &AU_PROFILE[i];
        b = &AU_PROFILE[i + 1];
        if (v < a->v || v >= b->v) continue;
        float f = (v - a->v) / (b->v - a->v);
        *hz     = (int)(a->freq_hz + f * (float)(b->freq_hz - a->freq_hz));
        *on_ms  = (int)(a->on_ms  + f * (float)(b->on_ms  - a->on_ms));
        *off_ms = (int)(a->off_ms + f * (float)(b->off_ms - a->off_ms));
        return;
    }
}

/* ---- phase scheduler ------------------------------------------------- */

typedef struct {
    bool active;        /* climb-audio state engaged (fresh cycle started) */
    bool on_phase;      /* true = inside the ON window of the current cycle */
    double phase_end;   /* monotonic t_s at which the current phase ends */
} au_sched_t;

typedef struct {
    bool silent;        /* tone off this instant: sink / fault / not latched */
    bool on;            /* duty high (inside ON window); valid when !silent */
    int hz;             /* target pitch this instant */
} au_out_t;

static inline void au_sched_init(au_sched_t *s)
{
    s->active = false;
    s->on_phase = false;
    s->phase_end = 0.0;
}

/* One nonblocking step.
 *   t_s    monotonic timestamp (s)
 *   climb  vario_core latch output (1 = climb audio allowed)
 *   v      current climb rate (m/s); <= 0 stays silent
 * Param changes never restart the phase timeline: ON/OFF durations are
 * re-read from the interpolated profile only when a phase boundary is
 * crossed, and the boundary chain is anchored to the ORIGINAL cycle
 * start (+=), so cadence neither drifts nor glitches. */
static inline void au_sched_step(au_sched_t *s, double t_s, bool climb,
                                 float v, au_out_t *o)
{
    if (!climb || v <= 0.f) {
        s->active = false;
        o->silent = true;
        o->on = false;
        o->hz = 0;
        return;
    }

    int hz, on_ms, off_ms;
    au_profile(v, &hz, &on_ms, &off_ms);
    o->silent = false;
    o->hz = hz;

    if (!s->active) {
        /* fresh entry: start the ON window now */
        s->active = true;
        s->on_phase = true;
        s->phase_end = t_s + (double)on_ms / 1000.0;
    } else {
        /* advance across ALL elapsed boundaries; each new phase takes
         * the current interpolated duration, anchored to the chain */
        while (t_s >= s->phase_end) {
            s->on_phase = !s->on_phase;
            double dur = (s->on_phase ? (double)on_ms : (double)off_ms)
                         / 1000.0;
            s->phase_end += dur;
        }
    }
    o->on = s->on_phase;
}

#endif /* AUDIO_SCHED_H */
