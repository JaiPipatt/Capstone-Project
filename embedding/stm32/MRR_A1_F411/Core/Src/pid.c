#include "pid.h"

void Pid_Init(PidCtrl *p, float kp, float ki, float kd,
              float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->integ = 0.0f;
    p->prev_meas = 0.0f;
    p->has_prev = 0U;
}

void Pid_Reset(PidCtrl *p)
{
    p->integ = 0.0f;
    p->has_prev = 0U;
}

float Pid_Update(PidCtrl *p, float setpoint, float meas, float dt)
{
    const float err = setpoint - meas;

    /* Derivative on measurement (no kick when the setpoint jumps) */
    float d_term = 0.0f;
    if (p->has_prev && p->kd != 0.0f) {
        d_term = -p->kd * (meas - p->prev_meas) / dt;
    }
    p->prev_meas = meas;
    p->has_prev = 1U;

    const float integ_new = p->integ + p->ki * err * dt;
    const float u_raw = p->kp * err + integ_new + d_term;

    float u = u_raw;
    if (u > p->out_max)      u = p->out_max;
    else if (u < p->out_min) u = p->out_min;

    /* Anti-windup (conditional integration):
     * keep integrating if not saturated, or if the error is pulling the
     * output back out of saturation. */
    if (u == u_raw ||
        (u_raw > p->out_max && err < 0.0f) ||
        (u_raw < p->out_min && err > 0.0f)) {
        p->integ = integ_new;
    }
    return u;
}
