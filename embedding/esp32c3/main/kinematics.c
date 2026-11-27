#include "kinematics.h"

static float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

bool kin_wheels_valid(const KinParams *p)
{
    return (p->wheel_radius_m > 0.0f) && (p->track_width_m > 0.0f);
}

void kin_wheels_ik(const KinParams *p, float v, float w, float *wl, float *wr)
{
    const float half_l = 0.5f * p->track_width_m;
    *wl = (v - w * half_l) / p->wheel_radius_m;
    *wr = (v + w * half_l) / p->wheel_radius_m;
}

void kin_wheels_fk(const KinParams *p, float wl, float wr, float *v, float *w)
{
    *v = p->wheel_radius_m * 0.5f * (wr + wl);
    *w = p->wheel_radius_m * (wr - wl) / p->track_width_m;
}

void kin_pantilt_ik(const KinParams *p, float pan, float tilt, float *q3, float *q4)
{
    pan  = clampf(pan,  p->pan_min,  p->pan_max);
    tilt = clampf(tilt, p->tilt_min, p->tilt_max);

    const float a = tilt / p->k_tilt;     /* common-mode  (both gears same direction)   */
    const float b = pan  / p->k_pan;      /* differential (gears opposite directions)   */
    *q3 = a + b;
    *q4 = a - b;
}

void kin_pantilt_fk(const KinParams *p, float q3, float q4, float *pan, float *tilt)
{
    *tilt = p->k_tilt * 0.5f * (q3 + q4);
    *pan  = p->k_pan  * 0.5f * (q3 - q4);
}
