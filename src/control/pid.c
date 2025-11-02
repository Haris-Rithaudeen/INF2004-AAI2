#include "control/pid.h"

// clamp helper
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void pid_init(pid_t* p, float kp, float ki, float kd, float dt_s,
              float out_min, float out_max, float integ_min, float integ_max) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->dt_s = dt_s;
    p->out_min = out_min; p->out_max = out_max;
    p->integ_min = integ_min; p->integ_max = integ_max;
    pid_reset(p);
}

void pid_reset(pid_t* p) {
    p->integ = 0.0f;
    p->prev_err = 0.0f;
    p->first = 1;
}

void pid_set_gains(pid_t* p, float kp, float ki, float kd) {
    p->kp = kp; p->ki = ki; p->kd = kd;
}

float pid_update(pid_t* p, float setpoint, float measurement) {
    float err = setpoint - measurement;

    // derivative (on error)
    float derr = p->first ? 0.0f : (err - p->prev_err) / p->dt_s;

    // integrator with clamping (simple anti-windup)
    p->integ += err * p->dt_s;
    p->integ = clampf(p->integ, p->integ_min, p->integ_max);

    float u = p->kp * err + p->ki * p->integ + p->kd * derr;
    p->prev_err = err;
    p->first = 0;

    return clampf(u, p->out_min, p->out_max);
}
