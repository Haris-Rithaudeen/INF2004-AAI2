#ifndef SERVO_H
#define SERVO_H
#pragma once
#include "pico/stdlib.h"
#include <stdbool.h>

// Initialize servo motor on the configured GPIO pin
void servo_init(void);

// Set servo angle (0-180 degrees)
// 0° = full left, 90° = center, 180° = full right
void servo_set_angle(uint8_t angle);

// Get current servo angle
uint8_t servo_get_angle(void);

// Set servo to center position (90°)
void servo_center(void);

#endif