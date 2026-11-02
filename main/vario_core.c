/* vario_core.c — implementation; see vario_core.h for the contract.
 *
 * The update math mirrors the Python reference step for step (same
 * operation order), so the double-precision build reproduces the Python
 * float64 outputs to machine tolerance and the float build stays within
 * the documented VARIO_FLOAT_TOLERANCE.
 */
#include "vario_core.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/* estimator                                                           */
/* ------------------------------------------------------------------ */
void vario_est_init(vario_est_state_t *st, const vario_est_config_t *cfg)
{
    st->h = (vario_real_t)0.0;
    st->v = (vario_real_t)0.0;
    st->p00 = (vario_real_t)1.0;
    st->p01 = (vario_real_t)0.0;
    st->p11 = (vario_real_t)1.0;
    st->q_eff = cfg->q;
    st->t_prev = 0.0;
    st->initialized = 0;
    st->n_gated = 0;
    st->n_rejected = 0;
}

vario_real_t vario_est_update(vario_est_state_t *st,
                              const vario_est_config_t *cfg,
                              vario_real_t z_m, double t_s)
{
    if (!st->initialized) {
        st->initialized = 1;
        st->t_prev = t_s;
        st->h = z_m;
        st->v = (vario_real_t)0.0;
        return st->v;
    }
    double dt_d = t_s - st->t_prev;
    if (dt_d <= 0.0)               /* duplicate/out-of-order: hold state */
        return st->v;
    st->t_prev = t_s;
    vario_real_t dt = (vario_real_t)dt_d;

    /* --- prediction (cv_model: F=[[1,dt],[0,1]], H=[1,0],
         Q = q_eff * [[dt^4/4, dt^3/2],[dt^3/2, dt^2]]) --- */
    vario_real_t xp0 = st->h + st->v * dt;
    vario_real_t xp1 = st->v;
    vario_real_t q = st->q_eff;
    vario_real_t q00 = q * (dt * dt * dt * dt / (vario_real_t)4.0);
    vario_real_t q01 = q * (dt * dt * dt / (vario_real_t)2.0);
    vario_real_t q11 = q * (dt * dt);
    vario_real_t pp00 = st->p00 + (vario_real_t)2.0 * dt * st->p01
                        + dt * dt * st->p11 + q00;
    vario_real_t pp01 = st->p01 + dt * st->p11 + q01;
    vario_real_t pp11 = st->p11 + q11;

    /* --- plausibility SATURATION (never replacement) --- */
    vario_real_t max_dev = cfg->v_plaus_max * dt + cfg->plaus_margin_m;
    vario_real_t dev = z_m - xp0;
    if (dev > max_dev || dev < -max_dev) {
        st->n_rejected++;
        dev = (dev > (vario_real_t)0.0) ? max_dev : -max_dev;
        z_m = xp0 + dev;
    }

    /* --- innovation gate --- */
    vario_real_t y = z_m - xp0;                  /* == clamped dev */
    vario_real_t S = pp00 + cfg->r;
#ifdef VARIO_USE_DOUBLE
    vario_real_t sig = (vario_real_t)sqrt((double)S);
#else
    vario_real_t sig = sqrtf(S);
#endif
    if (y > cfg->gate_sigma * sig ||
        y < -cfg->gate_sigma * sig) {            /* reject: predict only */
        st->n_gated++;
        st->h = xp0;
        st->v = xp1;
        st->p00 = pp00;
        st->p01 = pp01;
        st->p11 = pp11;
    } else {                                     /* correct */
        vario_real_t k0 = pp00 / S;
        vario_real_t k1 = pp01 / S;
        st->h = xp0 + k0 * y;
        st->v = xp1 + k1 * y;
        /* P = (I - K H) Pp, H = [1, 0] */
        st->p00 = pp00 - k0 * pp00;
        st->p01 = pp01 - k0 * pp01;
        st->p11 = pp11 - k1 * pp01;
    }

    /* --- motion-adaptive process noise --- */
    {
        vario_real_t av = st->v;
        if (av < (vario_real_t)0.0) av = -av;
        vario_real_t s = av / cfg->v_scale;
        if (s > (vario_real_t)3.0) s = (vario_real_t)3.0;
        st->q_eff = cfg->q *
            ((vario_real_t)1.0 + cfg->q_gain * s);
    }
    return st->v;
}

/* ------------------------------------------------------------------ */
/* audio                                                               */
/* ------------------------------------------------------------------ */
void vario_audio_init(vario_audio_state_t *st)
{
    st->state = 0;
    st->on_since = 0.0;
    st->off_pending = -1.0;
    st->n_on = st->n_off = st->n_fault_veto = st->n_max_on = 0;
}

int vario_audio_update(vario_audio_state_t *st,
                       const vario_audio_config_t *cfg,
                       vario_real_t v, double t_s, int fault)
{
    if (fault) {
        if (st->state == 1) {
            st->state = 0;
            st->n_off++;
            st->n_fault_veto++;
            st->off_pending = -1.0;
        }
        return st->state;
    }
    if (st->state == 0) {
        if (v >= cfg->lift_on) {
            st->state = 1;
            st->on_since = t_s;
            st->off_pending = -1.0;
            st->n_on++;
        }
    } else {
        if (cfg->max_on_s > (vario_real_t)0.0 &&
            t_s - st->on_since >= (double)cfg->max_on_s) {
            st->state = 0;
            st->n_off++;
            st->n_max_on++;
            st->off_pending = -1.0;
        } else if (v < cfg->lift_off) {
            if (st->off_pending < 0.0)
                st->off_pending = t_s;
            {
                double held = t_s - st->off_pending;
                int min_on_ok = (t_s - st->on_since) >= (double)cfg->min_on_s;
                if (held >= (double)cfg->hold_s && min_on_ok) {
                    st->state = 0;
                    st->n_off++;
                    st->off_pending = -1.0;
                }
            }
        } else {
            st->off_pending = -1.0;
        }
    }
    return st->state;
}

/* ------------------------------------------------------------------ */
/* fault detector                                                      */
/* ------------------------------------------------------------------ */
void vario_fault_init(vario_fault_t *f, vario_real_t v_plaus_max,
                      vario_real_t margin_m, int trip_n, int clear_n)
{
    f->v_plaus_max = v_plaus_max;
    f->margin_m = margin_m;
    f->trip_n = trip_n;
    f->clear_n = clear_n;
    f->bad = f->good = f->flag = 0;
}

int vario_fault_update(vario_fault_t *f, vario_real_t z, vario_real_t h_pred,
                       vario_real_t dt)
{
    vario_real_t dev = z - h_pred;
    vario_real_t bound = f->v_plaus_max * dt + f->margin_m;
    if (dev > bound || dev < -bound) {
        f->bad++;
        f->good = 0;
        if (f->bad >= f->trip_n)
            f->flag = 1;
    } else {
        f->good++;
        f->bad = 0;
        if (f->good >= f->clear_n)
            f->flag = 0;
    }
    return f->flag;
}
