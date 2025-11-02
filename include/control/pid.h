#pragma once
#include <stdint.h>

typedef struct {
    // gains
    float kp, ki, kd;
    // state
    float integ;
    float prev_err;
    // limits
    float out_min, out_max;
    float integ_min, integ_max;
    // timing
    float dt_s;
    // misc
    uint8_t first;
} pid_t;

void  pid_init(pid_t* p, float kp, float ki, float kd, float dt_s,
               float out_min, float out_max, float integ_min, float integ_max);
void  pid_reset(pid_t* p);
void  pid_set_gains(pid_t* p, float kp, float ki, float kd);
float pid_update(pid_t* p, float setpoint, float measurement);

