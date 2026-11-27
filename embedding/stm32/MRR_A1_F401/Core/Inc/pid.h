#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float integ;        /* integral term already multiplied by ki */
    float prev_meas;
    uint8_t has_prev;
} PidCtrl;

void  Pid_Init(PidCtrl *p, float kp, float ki, float kd,
               float out_min, float out_max);

/* Clear the integral and the derivative memory (use when the mode changes). */
void  Pid_Reset(PidCtrl *p);

/* One PID step. Call at a FIXED rate; dt is that period in seconds. */
float Pid_Update(PidCtrl *p, float setpoint, float meas, float dt);

#endif /* PID_H */
