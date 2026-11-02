/* madgwick.c — operation-order-preserving port of attitude.py. */
#include "madgwick.h"

#include <math.h>
#include <string.h>

#define MW_EPS 1e-5

static void qmul(const double a[4], const double b[4], double out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* predicted gravity direction in BODY coords: q^-1 (0,0,1) q */
static void grav_pred_body(const double q[4], double out[3])
{
    double qc[4] = {q[0], -q[1], -q[2], -q[3]};
    double g4[4] = {0.0, 0.0, 0.0, 1.0};
    double t[4], r[4];
    qmul(qc, g4, t);
    qmul(t, q, r);
    out[0] = r[1]; out[1] = r[2]; out[2] = r[3];
}

static void normalize4(double v[4])
{
    double n = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
    if (n < 1e-12) return;
    for (int i = 0; i < 4; i++) v[i] /= n;
}

void madgwick_update(double q[4], const double a[3], const double g[3],
                     double dt, double beta)
{
    double an = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    double qn[4];
    if (an < 1e-9) {                       /* free fall: gyro only */
        double w4[4] = {0.0, g[0], g[1], g[2]};
        double t[4];
        qmul(q, w4, t);
        for (int i = 0; i < 4; i++) qn[i] = q[i] + 0.5 * t[i] * dt;
        normalize4(qn);
        memcpy(q, qn, sizeof(qn));
        return;
    }
    double ad[3];
    for (int i = 0; i < 3; i++) ad[i] = -a[i] / an;   /* accel_sign = -1 */

    double f[3];
    grav_pred_body(q, f);
    f[0] -= ad[0]; f[1] -= ad[1]; f[2] -= ad[2];

    /* numeric Jacobian (central differences on the unit-quaternion manifold) */
    double step[4] = {0, 0, 0, 0};
    for (int j = 0; j < 4; j++) {
        double qp[4], qm[4], pp[3], pm[3];
        memcpy(qp, q, sizeof(qp));
        memcpy(qm, q, sizeof(qm));
        qp[j] += MW_EPS; normalize4(qp);
        qm[j] -= MW_EPS; normalize4(qm);
        grav_pred_body(qp, pp);
        grav_pred_body(qm, pm);
        for (int i = 0; i < 3; i++) {
            double jcol = (pp[i] - pm[i]) / (2.0 * MW_EPS);
            step[j] += jcol * f[i];
        }
    }
    double sn = sqrt(step[0]*step[0] + step[1]*step[1] +
                     step[2]*step[2] + step[3]*step[3]);
    if (sn > 1e-12) {
        for (int i = 0; i < 4; i++) step[i] /= sn;
    }
    double w4[4] = {0.0, g[0], g[1], g[2]};
    double t[4];
    qmul(q, w4, t);
    for (int i = 0; i < 4; i++) {
        qn[i] = q[i] + (0.5 * t[i] - beta * step[i]) * dt;
    }
    normalize4(qn);
    memcpy(q, qn, sizeof(qn));
}

void quat_from_accel_tilt(const double a[3], double q[4])
{
    double n = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (n < 1e-6) { q[0] = 1.0; q[1] = q[2] = q[3] = 0.0; return; }
    double u[3] = {a[0]/n, a[1]/n, a[2]/n};
    double d = -u[2];                          /* dot(u, (0,0,-1)) */
    if (d < -0.999999) {                       /* 180 deg: any perpendicular */
        double ax[3] = {0.0, -u[2], u[1]};     /* cross(u, (1,0,0)) */
        double an = sqrt(ax[0]*ax[0] + ax[1]*ax[1] + ax[2]*ax[2]);
        if (an < 1e-6) { ax[0] = u[2]; ax[1] = 0.0; ax[2] = -u[0];
                         an = sqrt(ax[0]*ax[0] + ax[1]*ax[1] + ax[2]*ax[2]); }
        q[0] = 0.0; q[1] = ax[0]/an; q[2] = ax[1]/an; q[3] = ax[2]/an;
        return;
    }
    double w[3] = {-u[1], u[0], 0.0};          /* cross(u, (0,0,-1)) */
    q[0] = 1.0 + d; q[1] = w[0]; q[2] = w[1]; q[3] = w[2];
    normalize4(q);
}

void accel_vertical_body_frame(const double q[4], double out[3])
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    out[0] = 2.0 * (x * z - w * y);
    out[1] = 2.0 * (y * z + w * x);
    out[2] = 1.0 - 2.0 * (x * x + y * y);
}
