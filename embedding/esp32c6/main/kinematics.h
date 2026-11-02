#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>

typedef struct {
    float wheel_radius_m;
    float track_width_m;
    float k_tilt, k_pan;
    float pan_min, pan_max, tilt_min, tilt_max;
} KinParams;

bool kin_wheels_valid(const KinParams *p);

/* Outer wheels (differential drive).
 *   v  [m/s]   forward speed        w [rad/s] turn rate (counter-clockwise +)
 *   wl, wr [rad/s] angular speed of left / right wheel (positive = forward) */
void kin_wheels_ik(const KinParams *p, float v, float w, float *wl, float *wr);
void kin_wheels_fk(const KinParams *p, float wl, float wr, float *v, float *w);

/* Inner differential (pan / tilt).  q3, q4 in rad (M3_IL, M4_IR).
 * IK clamps pan/tilt to the limits in KinParams first. */
void kin_pantilt_ik(const KinParams *p, float pan, float tilt, float *q3, float *q4);
void kin_pantilt_fk(const KinParams *p, float q3, float q4, float *pan, float *tilt);

#endif /* KINEMATICS_H */
