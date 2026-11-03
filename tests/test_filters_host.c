/* test_filters_host.c — host runner for the filter bank (no ESP-IDF deps).
 *
 * Feeds a deterministic synthetic altitude profile through all four C
 * filter ports and prints "i,t,z,C03,D01,S4,S5" CSV.  The companion Python
 * script (tests/test_filters_ref.py) regenerates the same profile, runs the
 * original benchmark reference filters, and compares within float tolerance.
 *
 * Build (MSVC, from vcvars64 prompt):  cl /TC /DVARIO_USE_DOUBLE
 *      tests/test_filters_host.c main/filters.c main/vario_core.c
 */
#include <math.h>
#include <stdio.h>
#include "filters.h"

#define FS_HZ 40.0
#define N ((int)(20.0 * FS_HZ))   /* 20 s profile */

/* deterministic synthetic profile — MUST stay in sync with the Python side */
static double profile_z(double t)
{
    double w = 0.05 * sin(2.0 * 3.14159265358979323846 * 0.7 * t)
             + 0.02 * sin(2.0 * 3.14159265358979323846 * 3.1 * t);
    if (t < 5.0)  return w;
    if (t < 10.0) return 1.2 * (t - 5.0) + w;
    if (t < 10.5) return 6.0 + w;
    return 6.0 - 4.9 * (1.0 - exp(-(t - 10.5) / 0.15)) + w;
}

int main(void)
{
    static char states[FILT_N][256];
    for (int f = 0; f < FILT_N; f++)
        g_filters[f].reset(states[f]);
    printf("i,t,z,C03,D01,S4,S5\n");
    for (int i = 0; i < N; i++) {
        double t = i / FS_HZ;
        float z = (float)profile_z(t);
        double out[FILT_N];
        for (int f = 0; f < FILT_N; f++)
            out[f] = g_filters[f].update(states[f], z, t);
        printf("%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
               i, t, z, out[FILT_C03], out[FILT_D01], out[FILT_S4], out[FILT_S5]);
    }
    return 0;
}
