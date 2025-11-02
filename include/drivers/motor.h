#pragma once
#include <stdint.h>
#include <stdbool.h>

// Cytron Robo Pico on-board drivers using two GPIOs per motor (A/B channels on same PWM slice).
// Direction convention:
//  - anticlockwise=false ("clockwise"): A = PWM, B = LOW
//  - anticlockwise=true  ("anticlock"): B = PWM, A = LOW

void motor_init_all(void);
void motor_set_speed_percent(uint speed_percent, bool anticlockwise);
void motor_stop(void);

void motor_set_signed(float right_percent, float left_percent);