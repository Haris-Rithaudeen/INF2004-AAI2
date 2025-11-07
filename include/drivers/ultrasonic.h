#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#pragma once
#include "pico/stdlib.h"
#include <stdbool.h>

// Initialize ultrasonic sensor
void ultrasonic_init(void);

// Measure distance in centimeters
// Returns NAN if out of range or timeout
float ultrasonic_measure_cm(void);

// Measure distance and check if within valid range
// Returns true if valid, false if out of range/timeout
bool ultrasonic_measure_valid(float* distance_cm);

// Check if object is within specified distance
bool ultrasonic_is_object_within(float threshold_cm);

// Get averaged distance over multiple samples
float ultrasonic_measure_averaged_cm(int samples);

// Print ultrasonic sensor data
void ultrasonic_print_data(void);

// Scan for obstacle edges (used with servo)
// Returns width estimate in cm (distance between clear sides)
typedef struct {
    float left_distance_cm;
    float center_distance_cm;
    float right_distance_cm;
    bool left_clear;
    bool right_clear;
    float estimated_width_cm;
} obstacle_scan_t;

#endif