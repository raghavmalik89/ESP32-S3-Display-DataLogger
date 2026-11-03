/* filters.c — see filters.h.  Ports follow the Python references step for
 * step (same operation order) so double-precision host builds track the
 * benchmark within float tolerance; the firmware float build uses the same
 * code with float state.
 *
 * Shared model pieces (from tracking.py):
 *   cv_model(dt,q,r): F=[[1,dt],[0,1]], H=[1,0],
 *        Q = q*[[dt^4/4, dt^3/2],[dt^3/2, dt^2]]
 *   alpha-beta gains (critically damped family, response time tau):
 *        alpha = 1-exp(-dt/tau); beta = alpha^2/(2-alpha)
 *   steady_state_gains: Riccati iterated 200x from P=I*10, frozen.
 */
#include "filters.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DT_RECOMP_TOL 0.02f

/* ------------------------------------------------------------------ */
/* D01 — full 2-state CV Kalman (kalman.py::D01KalmanCV, q=1, r=0.035) */
/* ------------------------------------------------------------------ */
typedef struct {
    float h, v;
    float p00, p01, p11;
    int init;
    double t_prev;
} d01_state_t;

static void d01_reset(void *s_)
{
    d01_state_t *s = (d01_state_t *)s_;
    memset(s, 0, sizeof(*s));
    s->p00 = 10.f; s->p11 = 10.f;
}

static float d01_step(d01_state_t *s, float z, double t, float q, float r)
{
    if (!s->init) {
        s->init = 1; s->t_prev = t; s->h = z; s->v = 0.f;
        return 0.f;
    }
    double dtd = t - s->t_prev;
    if (dtd <= 0.0) return s->v;
    s->t_prev = t;
    float dt = (float)dtd;

    float xp0 = s->h + s->v * dt, xp1 = s->v;
    float q00 = q * (dt * dt * dt * dt / 4.f);
    float q01 = q * (dt * dt * dt / 2.f);
    float q11 = q * (dt * dt);
    float pp00 = s->p00 + 2.f * dt * s->p01 + dt * dt * s->p11 + q00;
    float pp01 = s->p01 + dt * s->p11 + q01;
    float pp11 = s->p11 + q11;
    float S = pp00 + r;
    float k0 = pp00 / S, k1 = pp01 / S;
    float y = z - xp0;
    s->h = xp0 + k0 * y;
    s->v = xp1 + k1 * y;
    s->p00 = pp00 - k0 * pp00;
    s->p01 = pp01 - k0 * pp01;
    s->p11 = pp11 - k1 * pp01;
    return s->v;
}

#define D01_DEF_Q 1.0f
#define D01_DEF_R 0.035f

static float d01_update(void *s_, float z, double t)
{
    return d01_step((d01_state_t *)s_, z, t, D01_DEF_Q, D01_DEF_R);
}

static float d01_alt(const void *s_) { return ((const d01_state_t *)s_)->h; }

/* ------------------------------------------------------------------ */
/* C03 — steady-state CV tracker (tracking.py::C03CVTracker, q=1,r=.035) */
/* Gains: Riccati converged ONCE at the first physical dt, then frozen; */
/* F still uses the actual dt.  Refit only if dt shifts > 2%.         */
/* ------------------------------------------------------------------ */
typedef struct {
    float h, v;
    float k0, k1;
    float dt_fit;
    int init;
    double t_prev;
} c03_state_t;

static void c03_reset(void *s_)
{
    c03_state_t *s = (c03_state_t *)s_;
    memset(s, 0, sizeof(*s));
}

/* iterate the Riccati to convergence (200 fixed iters, like the reference) */
static void c03_refit(c03_state_t *s, float dt)
{
    float q = D01_DEF_Q, r = D01_DEF_R;
    float p00 = 10.f, p01 = 0.f, p11 = 10.f;
    for (int i = 0; i < 200; i++) {
        float q00 = q * (dt * dt * dt * dt / 4.f);
        float q01 = q * (dt * dt * dt / 2.f);
        float q11 = q * (dt * dt);
        float pp00 = p00 + 2.f * dt * p01 + dt * dt * p11 + q00;
        float pp01 = p01 + dt * p11 + q01;
        float pp11 = p11 + q11;
        float S = pp00 + r;
        float k0 = pp00 / S, k1 = pp01 / S;
        p00 = pp00 - k0 * pp00;
        p01 = pp01 - k0 * pp01;
        p11 = pp11 - k1 * pp01;
    }
    float q00 = q * (dt * dt * dt * dt / 4.f);
    float q01 = q * (dt * dt * dt / 2.f);
    float q11 = q * (dt * dt);
    float pp00 = p00 + 2.f * dt * p01 + dt * dt * p11 + q00;
    float pp01 = p01 + dt * p11 + q01;
    float pp11 = p11 + q11;
    float S = pp00 + r;
    s->k0 = pp00 / S;
    s->k1 = pp01 / S;
    s->dt_fit = dt;
}

static float c03_update(void *s_, float z, double t)
{
    c03_state_t *s = (c03_state_t *)s_;
    if (!s->init) {
        s->init = 1; s->t_prev = t; s->h = z; s->v = 0.f; s->dt_fit = 0.f;
        return 0.f;
    }
    double dtd = t - s->t_prev;
    if (dtd <= 0.0) return s->v;
    s->t_prev = t;
    float dt = (float)dtd;
    if (s->dt_fit == 0.f ||
        fabsf(dt - s->dt_fit) / s->dt_fit > DT_RECOMP_TOL)
        c03_refit(s, dt);
    float y = z - (s->h + s->v * dt);
    s->h = s->h + s->v * dt + s->k0 * y;
    s->v = s->v + s->k1 * y;
    return s->v;
}

static float c03_alt(const void *s_) { return ((const c03_state_t *)s_)->h; }

/* ------------------------------------------------------------------ */
/* S4 — fast/slow gated dual estimator                                */
/* (stacked.py::S4FastSlowGated, Phase-4 WP-B/D3 corrected version)   */
/* ------------------------------------------------------------------ */
#define S4_TAU_FAST   0.35f
#define S4_Q_SLOW     0.3f
#define S4_R_SLOW     0.035f
#define S4_V_SCALE    0.8f
#define S4_EXIT_V     0.12f
#define S4_EXIT_HOLD  0.25f
#define S4_W_FLOOR    0.3f
#define S4_V_PLAUS    8.0f
#define S4_PLAUS_M    0.3f

typedef struct {
    /* fast branch: C01 alpha-beta */
    float fh, fv, fa, fb, fdt;
    /* slow branch: D01 */
    d01_state_t slow;
    /* latch */
    int fast_hold;
    double below_since;
    double t_prev;
    int init;
} s4_state_t;

static void s4_reset(void *s_)
{
    s4_state_t *s = (s4_state_t *)s_;
    memset(s, 0, sizeof(*s));
    d01_reset(&s->slow);
    s->below_since = -1.0;
}

static float s4_update(void *s_, float z, double t)
{
    s4_state_t *s = (s4_state_t *)s_;
    if (!s->init) {
        s->init = 1; s->t_prev = t;
        s->fh = z; s->fv = 0.f; s->fdt = 0.f;
        d01_reset(&s->slow);
        d01_step(&s->slow, z, t, S4_Q_SLOW, S4_R_SLOW);
        return 0.f;
    }
    double dtd = t - s->t_prev;
    if (dtd <= 0.0) return s->slow.v;
    s->t_prev = t;
    float dt = (float)dtd;

    /* plausibility SATURATE (never replace) on the fast prediction */
    float h_pred = s->fh + s->fv * dt;
    float max_dev = S4_V_PLAUS * dt + S4_PLAUS_M;
    float dev = z - h_pred;
    float zc = z;
    if (fabsf(dev) > max_dev)
        zc = h_pred + (dev > 0.f ? max_dev : -max_dev);

    /* fast branch update (alpha-beta, gains refit if dt shifts > 2%) */
    if (s->fdt == 0.f || fabsf(dt - s->fdt) / s->fdt > DT_RECOMP_TOL) {
        float a = 1.f - expf(-dt / S4_TAU_FAST);
        s->fa = a;
        s->fb = a * a / (2.f - a);
        s->fdt = dt;
    }
    float y = zc - (s->fh + s->fv * dt);
    s->fh = s->fh + s->fv * dt + s->fa * y;
    s->fv = s->fv + (s->fb / dt) * y;
    float vf = s->fv;

    /* slow branch */
    float vs = d01_step(&s->slow, zc, t, S4_Q_SLOW, S4_R_SLOW);

    /* motion latch */
    if (!s->fast_hold && fabsf(vf) > S4_V_SCALE)
        s->fast_hold = 1;
    if (s->fast_hold) {
        if (fabsf(vf) < S4_EXIT_V) {
            if (s->below_since < 0.0) s->below_since = t;
            if (t - s->below_since >= S4_EXIT_HOLD) {
                s->fast_hold = 0;
                s->below_since = -1.0;
            }
        } else {
            s->below_since = -1.0;
        }
    }

    float w = fminf(fabsf(vf) / S4_V_SCALE, 1.f);
    if (s->fast_hold) {
        /* sign-disagreement: the slow branch owns the exit */
        if ((vf > 0.f) != (vs > 0.f) && fabsf(vs) < S4_EXIT_V)
            w = 0.f;
        else
            w = fmaxf(w, S4_W_FLOOR);
    }
    return (1.f - w) * vs + w * vf;
}

static float s4_alt(const void *s_) { return ((const s4_state_t *)s_)->slow.h; }

/* ------------------------------------------------------------------ */
/* S5 — light-gate robust CV KF (stacked.py::S5_LightGateKF), via      */
/* the already-ported vario_core estimator (identical algorithm).      */
/* ------------------------------------------------------------------ */
typedef struct {
    vario_est_state_t est;
    vario_est_config_t cfg;
} s5_state_t;

static const vario_est_config_t S5_CFG = {
    .q = 0.3f,
    .r = 0.035f,
    .gate_sigma = 3.5f,
    .q_gain = 2.0f,
    .v_scale = 1.0f,
    .v_plaus_max = 8.0f,
    .plaus_margin_m = 0.3f,
};

static void s5_reset(void *s_)
{
    s5_state_t *s = (s5_state_t *)s_;
    s->cfg = S5_CFG;
    vario_est_init(&s->est, &s->cfg);
}

/* vario_est_update already implements the Filter-base contract: first
 * sample initializes and returns 0; dt<=0 repeats the last estimate. */
static float s5_update(void *s_, float z, double t)
{
    s5_state_t *s = (s5_state_t *)s_;
    return (float)vario_est_update(&s->est, &s->cfg, z, t);
}

static float s5_alt(const void *s_) { return (float)((const s5_state_t *)s_)->est.h; }

/* ------------------------------------------------------------------ */
const filter_def_t g_filters[FILT_N] = {
    /* order = FILT_* indices = Vario-screen cycle order C03->D01->S4->S5 */
    { "C03", "C03_CV_tracker",    sizeof(c03_state_t), c03_reset, c03_update, c03_alt },
    { "D01", "D01_Kalman_CV",     sizeof(d01_state_t), d01_reset, d01_update, d01_alt },
    { "S4",  "S4_FastSlowGated",  sizeof(s4_state_t),  s4_reset,  s4_update,  s4_alt  },
    { "S5",  "S5_LightGateKF",    sizeof(s5_state_t),  s5_reset,  s5_update,  s5_alt  },
};

const char *filter_id(int f)
{
    return (f >= 0 && f < FILT_N) ? g_filters[f].id : "?";
}
