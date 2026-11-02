/* madgwick.h — C port of filter_benchmark/src/attitude.py (Madgwick IMU
 * update with numeric Jacobian + shortest-arc accel warm start). Body ->
 * Earth quaternion (w, x, y, z); Earth frame z-DOWN.
 */
#ifndef MADGWICK_H
#define MADGWICK_H

#ifdef __cplusplus
extern "C" {
#endif

/* One Madgwick IMU update. a = specific force (any units, normalized
 * inside), g = gyro rad/s, dt seconds (caller clamps), beta gradient gain.
 * q in/out unit quaternion. */
void madgwick_update(double q[4], const double a[3], const double g[3],
                     double dt, double beta);

/* Tilt-only warm start: align measured specific force onto (0,0,-g). */
void quat_from_accel_tilt(const double a[3], double q[4]);

/* Third row of R(body->earth): Earth-z axis in body coords. */
void accel_vertical_body_frame(const double q[4], double out[3]);

#ifdef __cplusplus
}
#endif

#endif
