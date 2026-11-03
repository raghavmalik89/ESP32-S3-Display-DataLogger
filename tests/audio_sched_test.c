/* audio_sched_test.c — deterministic host test of the Audio Rev 2 beep
 * scheduler (main/audio_sched.h + main/audio_engine.h) with the
 * production vario_core.c latch compiled in.  No ESP-IDF, no audio
 * hardware: a virtual PWM output is sampled at the production 40 Hz
 * cadence and edge timings are measured against the profile table.
 *
 * Build (see tools/win/sched_test.bat):
 *   zig cc -O2 -std=c99 --target=x86_64-windows-gnu -I main \
 *      tests/audio_sched_test.c main/vario_core.c -o audio_sched_test.exe
 *
 * Scenarios:
 *   A  constant climbs +0.3/+1/+2/+3/+5 m/s, 8 s each
 *   B  fluctuation around +2.0 m/s (+/-0.15 sine, 2 s period), 12 s
 *   C  ramp 0.3 -> 5.0 m/s over 12 s (pitch monotonicity)
 *   D  filter-switch step: +2.0 constant, step to +3.0 at t=5 s
 *   E  sink veto: +2.0 for 3 s, then -1.5 for 2 s, back to +2.0
 *   F  fault veto: climbing, fault=1 injected for 1 s
 *   G  hysteresis wobble: 0.15/0.25 m/s alternation (latch stays),
 *      then 0.03/0.15 (never latches)
 *
 * Tolerances: phase boundaries are detected at the first sample on/after
 * the true boundary, so measured durations land in [expected,
 * expected + DT].  Allowed window: [-1, +2*DT] with DT = 25 ms, plus
 * interpolation spread for scenarios with varying v.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_engine.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DT_MS     25.0
#define DT_S      (DT_MS / 1000.0)

static int s_failures = 0;

#define CHECK(cond, ...) do { \
    if (!(cond)) { s_failures++; \
        printf("  FAIL: " __VA_ARGS__); putchar('\n'); } \
} while (0)

/* ---- virtual PWM + edge measurement ---------------------------------- */

typedef struct {
    vario_audio_state_t lat;
    au_sched_t sched;
    bool audible;        /* !silent && on (previous sample) */
    /* edge log */
    double t_start[1024], t_end[1024];
    int n_beeps;
    double cur_start;
    bool in_on;
    /* pitch track */
    double hz_sum; int hz_n; int hz_min, hz_max;
} sim_t;

static void sim_init(sim_t *m)
{
    memset(m, 0, sizeof(*m));
    vario_audio_init(&m->lat);
    au_sched_init(&m->sched);
}

static const vario_audio_config_t CFG = {
    .lift_on = 0.20f,
    .lift_off = 0.05f,
    .hold_s = 0.25f,
    .min_on_s = 0.10f,
    .max_on_s = 30.0f,
};

/* one 40 Hz tick; returns engine output for assertion use */
static au_out_t sim_tick(sim_t *m, float v, double t_s, int fault)
{
    au_out_t o;
    au_engine_step(&m->lat, &CFG, &m->sched, v, t_s, fault, &o);
    bool aud = !o.silent && o.on;
    if (aud && !m->in_on) {            /* beep start edge */
        m->in_on = true;
        m->cur_start = t_s;
        if (m->n_beeps < 1024) m->t_start[m->n_beeps] = t_s;
    } else if (!aud && m->in_on) {     /* beep end edge */
        m->in_on = false;
        if (m->n_beeps < 1024) m->t_end[m->n_beeps] = t_s;
        m->n_beeps++;
    }
    if (!o.silent) {
        m->hz_sum += o.hz; m->hz_n++;
        if (o.hz < m->hz_min || m->hz_n == 1) m->hz_min = o.hz;
        if (o.hz > m->hz_max || m->hz_n == 1) m->hz_max = o.hz;
    }
    return o;
}

/* ---- per-scenario statistics ----------------------------------------- */

typedef struct {
    double on_min, on_max, on_sum; int on_n;
    double off_min, off_max, off_sum; int off_n;
    double per_min, per_max, per_sum; int per_n;
} stats_t;

static void stats_gather(const sim_t *m, stats_t *s)
{
    memset(s, 0, sizeof(*s));
    s->on_min = s->off_min = s->per_min = 1e9;
    s->on_max = s->off_max = s->per_max = -1e9;
    for (int i = 0; i + 1 < m->n_beeps; i++) {   /* complete pairs only */
        double on_ms  = (m->t_end[i] - m->t_start[i]) * 1000.0;
        double off_ms = (m->t_start[i + 1] - m->t_end[i]) * 1000.0;
        double per_ms = (m->t_start[i + 1] - m->t_start[i]) * 1000.0;
        if (on_ms < s->on_min) s->on_min = on_ms;
        if (on_ms > s->on_max) s->on_max = on_ms;
        s->on_sum += on_ms; s->on_n++;
        if (off_ms < s->off_min) s->off_min = off_ms;
        if (off_ms > s->off_max) s->off_max = off_ms;
        s->off_sum += off_ms; s->off_n++;
        if (per_ms < s->per_min) s->per_min = per_ms;
        if (per_ms > s->per_max) s->per_max = per_ms;
        s->per_sum += per_ms; s->per_n++;
    }
}

static void stats_print(const char *tag, const stats_t *s)
{
    if (s->on_n == 0) { printf("  %-4s: no complete beeps\n", tag); return; }
    printf("  %-4s: ON  %5.0f..%4.0f ms (mean %5.1f)  OFF %5.0f..%4.0f ms "
           "(mean %5.1f)  period %5.0f..%4.0f ms (mean %5.1f)  n=%d\n",
           tag, s->on_min, s->on_max, s->on_sum / s->on_n,
           s->off_min, s->off_max, s->off_sum / s->off_n,
           s->per_min, s->per_max, s->per_sum / s->per_n, s->on_n);
}

/* run `seconds` of constant climb v, starting the clock at t0 */
static void run_constant(sim_t *m, double t0, float v, double seconds)
{
    int steps = (int)(seconds / DT_S);
    for (int i = 0; i < steps; i++)
        sim_tick(m, v, t0 + i * DT_S, 0);
}

/* ---- scenario A: constant climbs ------------------------------------- */

static void scenario_constant(float v)
{
    char tag[32];
    snprintf(tag, sizeof(tag), "A%+.1f", v);
    printf("Scenario A: constant %+.1f m/s, 8 s\n", v);

    int ehz, eon, eoff;
    au_profile(v, &ehz, &eon, &eoff);
    printf("  expected: %d Hz, ON %d ms, OFF %d ms, period %d ms\n",
           ehz, eon, eoff, eon + eoff);

    sim_t m; sim_init(&m);
    run_constant(&m, 0.0, v, 8.0);
    stats_t s; stats_gather(&m, &s);
    stats_print(tag, &s);

    CHECK(m.n_beeps >= 4, "too few beeps: %d", m.n_beeps);
    if (s.on_n > 0) {
        /* Boundary detection quantizes both edges to the sample grid:
         * each measured duration/period lands within +/- 1 tick of the
         * nominal value (mean converges to nominal). */
        CHECK(s.on_min >= eon - DT_MS - 1, "ON below bound: min %.0f < %d",
              s.on_min, eon - (int)DT_MS - 1);
        CHECK(s.on_max <= eon + DT_MS + 1, "ON above bound: max %.0f > %d",
              s.on_max, eon + (int)DT_MS + 1);
        CHECK(s.off_min >= eoff - DT_MS - 1, "OFF below bound: min %.0f < %d",
              s.off_min, eoff - (int)DT_MS - 1);
        CHECK(s.off_max <= eoff + DT_MS + 1, "OFF above bound: max %.0f > %d",
              s.off_max, eoff + (int)DT_MS + 1);
        double per_mean = s.per_sum / s.per_n;
        CHECK(fabs(per_mean - (eon + eoff)) <= DT_MS + 1,
              "period mean %.1f vs %d", per_mean, eon + eoff);
    }
    if (m.hz_n > 0)
        CHECK(m.hz_min == ehz && m.hz_max == ehz,
              "pitch drift: %d..%d, expected %d", m.hz_min, m.hz_max, ehz);
}

/* ---- scenario B: fluctuation around +2 -------------------------------- */

static void scenario_fluctuation(void)
{
    printf("Scenario B: +2.0 m/s +/- 0.15 sine (2 s), 12 s\n");
    sim_t m; sim_init(&m);
    /* interpolation spread across the actual trace */
    int on_lo = 10000, on_hi = 0, off_lo = 10000, off_hi = 0;
    int steps = (int)(12.0 / DT_S);
    for (int i = 0; i < steps; i++) {
        double t = i * DT_S;
        float v = 2.0f + 0.15f * (float)sin(2.0 * M_PI * t / 2.0);
        int hz, on, off;
        au_profile(v, &hz, &on, &off);
        if (on < on_lo) on_lo = on;
        if (on > on_hi) on_hi = on;
        if (off < off_lo) off_lo = off;
        if (off > off_hi) off_hi = off;
        sim_tick(&m, v, t, 0);
    }
    stats_t s; stats_gather(&m, &s);
    stats_print("B", &s);
    printf("  interp spread over trace: ON %d..%d ms, OFF %d..%d ms\n",
           on_lo, on_hi, off_lo, off_hi);
    /* Defect detector: with the Rev-1 phase-reset bug, param changes at
     * nearly every sample truncated phases far below the interpolated
     * minimum.  With the fix every duration stays inside the spread. */
    if (s.on_n > 0) {
        CHECK(s.on_min >= on_lo - DT_MS - 1,
              "ON collapsed below interp spread: min %.0f (< %d)",
              s.on_min, on_lo - (int)DT_MS - 1);
        CHECK(s.on_max <= on_hi + 2 * DT_MS + 1,
              "ON overshoots interp spread: max %.0f (> %d)",
              s.on_max, on_hi + 2 * (int)DT_MS + 1);
        CHECK(s.off_min >= off_lo - DT_MS - 1,
              "OFF collapsed below interp spread: min %.0f (< %d)",
              s.off_min, off_lo - (int)DT_MS - 1);
        CHECK(s.off_max <= off_hi + 2 * DT_MS + 1,
              "OFF overshoots interp spread: max %.0f (> %d)",
              s.off_max, off_hi + 2 * (int)DT_MS + 1);
        /* cadence must continue throughout (no stall mid-scenario) */
        double per_mean = s.per_sum / s.per_n;
        CHECK(s.per_min > 0.5 * per_mean,
              "cadence stall: min period %.0f vs mean %.0f",
              s.per_min, per_mean);
    }
    CHECK(m.n_beeps >= 12 * 1000 / (on_hi + off_hi + 2 * (int)DT_MS),
          "too few beeps under fluctuation: %d", m.n_beeps);
}

/* ---- scenario C: ramp -------------------------------------------------- */

static void scenario_ramp(void)
{
    printf("Scenario C: ramp 0.3 -> 5.0 m/s over 12 s\n");
    sim_t m; sim_init(&m);
    int steps = (int)(12.0 / DT_S);
    int last_hz = -1, monotone_viol = 0;
    for (int i = 0; i < steps; i++) {
        double t = i * DT_S;
        float v = 0.3f + (5.0f - 0.3f) * (float)(t / 12.0);
        au_out_t o = sim_tick(&m, v, t, 0);
        if (!o.silent) {
            if (o.hz < last_hz) monotone_viol++;
            last_hz = o.hz;
        }
    }
    stats_t s; stats_gather(&m, &s);
    stats_print("C", &s);
    CHECK(monotone_viol == 0, "pitch not monotonic: %d violations", monotone_viol);
    CHECK(m.hz_min >= 500 && m.hz_max <= 1100,
          "pitch range %d..%d outside ramp expectation", m.hz_min, m.hz_max);
    CHECK(m.n_beeps >= 20, "too few beeps on ramp: %d", m.n_beeps);
}

/* ---- scenario D: filter-switch step mid-beep --------------------------- */

static void scenario_step(void)
{
    printf("Scenario D: +2.0 constant, step to +3.0 at t=5 s (filter switch)\n");
    sim_t m; sim_init(&m);
    au_out_t o = {0};
    double step_t = 5.0;
    int steps = (int)(10.0 / DT_S);
    int hz_before = 0, hz_after = 0, phase_at_step = -1;
    for (int i = 0; i < steps; i++) {
        double t = i * DT_S;
        float v = (t < step_t) ? 2.0f : 3.0f;
        o = sim_tick(&m, v, t, 0);
        if (!o.silent) {
            if (t < step_t) hz_before = o.hz;
            else if (hz_after == 0) hz_after = o.hz;
        }
        if (phase_at_step < 0 && t >= step_t) phase_at_step = o.on ? 1 : 0;
    }
    stats_t s; stats_gather(&m, &s);
    stats_print("D", &s);
    /* pitch must switch immediately at the step... */
    CHECK(hz_before == 670, "pre-step pitch %d != 670", hz_before);
    CHECK(hz_after == 810, "post-step pitch %d != 810 (immediate update)",
          hz_after);
    /* ...while the ON phase containing the step completes unmolested
     * (270 ms nominal at +2, allowed [270, 270+3*DT]) and the next
     * phases converge to the +3 profile (270/230) */
    if (s.on_n > 0) {
        CHECK(s.on_max <= 270 + DT_MS + 1,
              "phase containing step was disturbed: ON max %.0f", s.on_max);
    }
    if (s.off_n > 1) {
        /* last OFF gap must be the +3 profile (grid-quantized) */
        double last_off = (m.t_start[m.n_beeps - 1] - m.t_end[m.n_beeps - 2])
                          * 1000.0;
        CHECK(last_off >= 230 - DT_MS - 1 && last_off <= 230 + DT_MS + 1,
              "post-step OFF %.0f not converged to 230", last_off);
    }
    /* beeping must not have restarted: count continuity — at +2 for 5 s
     * (~8-9 beeps) plus +3 for 5 s (~10 beeps), no dropout gap > 400 ms */
    CHECK(m.n_beeps >= 15, "beeps lost across switch: %d", m.n_beeps);
}

/* ---- scenario E: sink veto --------------------------------------------- */

static void scenario_sink(void)
{
    printf("Scenario E: +2.0 (3 s), sink -1.5 (2 s), back to +2.0 (3 s)\n");
    sim_t m; sim_init(&m);
    run_constant(&m, 0.0, 2.0f, 3.0);
    int steps = (int)(2.0 / DT_S);
    double sil_at = -1;
    for (int i = 0; i < steps; i++) {
        double t = 3.0 + i * DT_S;
        au_out_t o = sim_tick(&m, -1.5f, t, 0);
        if (o.silent && sil_at < 0) sil_at = t;
    }
    int before = m.n_beeps;
    run_constant(&m, 5.0, 2.0f, 3.0);
    CHECK(sil_at >= 3.0 && sil_at <= 3.0 + DT_S + 0.001,
          "sink veto not immediate: silence at %.3f", sil_at);
    CHECK(m.n_beeps > before, "no beeps resumed after re-climb");
    stats_t s; stats_gather(&m, &s);
    stats_print("E", &s);
}

/* ---- scenario F: fault veto -------------------------------------------- */

static void scenario_fault(void)
{
    printf("Scenario F: +2.0 climbing, fault=1 for 1 s at t=2\n");
    sim_t m; sim_init(&m);
    int steps = (int)(5.0 / DT_S);
    double sil_at = -1, resume_at = -1;
    for (int i = 0; i < steps; i++) {
        double t = i * DT_S;
        int fault = (t >= 2.0 && t < 3.0) ? 1 : 0;
        au_out_t o = sim_tick(&m, 2.0f, t, fault);
        if (fault && o.silent && sil_at < 0) sil_at = t;
        if (!fault && !o.silent && sil_at > 0 && resume_at < 0) resume_at = t;
    }
    CHECK(sil_at >= 2.0 && sil_at <= 2.0 + DT_S + 0.001,
          "fault veto not immediate: silence at %.3f", sil_at);
    CHECK(resume_at >= 3.0 - 0.001 && resume_at <= 3.0 + 0.25 + 2 * DT_S,
          "resume not prompt: %.3f", resume_at);
    stats_t s; stats_gather(&m, &s);
    stats_print("F", &s);
}

/* ---- scenario G: hysteresis wobble -------------------------------------- */

static void scenario_wobble(void)
{
    printf("Scenario G: wobble 0.15/0.25 (latch holds), then 0.03/0.15 (never)\n");
    sim_t m; sim_init(&m);
    int steps = (int)(6.0 / DT_S);
    for (int i = 0; i < steps; i++) {
        float v = ((i / 12) % 2 == 0) ? 0.25f : 0.15f;   /* 300 ms each */
        sim_tick(&m, v, i * DT_S, 0);
    }
    int held_beeps = m.n_beeps;
    CHECK(held_beeps >= 5, "wobble around 0.20 broke the latch: %d beeps",
          held_beeps);

    sim_t m2; sim_init(&m2);
    for (int i = 0; i < steps; i++) {
        float v = ((i / 12) % 2 == 0) ? 0.15f : 0.03f;
        sim_tick(&m2, v, i * DT_S, 0);
    }
    CHECK(m2.n_beeps == 0, "sub-threshold wobble beeped: %d", m2.n_beeps);
    stats_t s; stats_gather(&m, &s);
    stats_print("G", &s);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("=== audio scheduler host test (Audio Rev 2) ===\n");
    printf("cadence %.0f ms/tick (production 40 Hz)\n\n", DT_MS);

    scenario_constant(0.3f);
    scenario_constant(1.0f);
    scenario_constant(2.0f);
    scenario_constant(3.0f);
    scenario_constant(5.0f);
    scenario_fluctuation();
    scenario_ramp();
    scenario_step();
    scenario_sink();
    scenario_fault();
    scenario_wobble();

    printf("\n=== %s: %d failure(s) ===\n",
           s_failures == 0 ? "PASS" : "FAIL", s_failures);
    return s_failures == 0 ? 0 : 1;
}
