#pragma once
// ==== Mechanical / hardware characteristics ====
#define ENCODER_CPR               20.0f
#define WHEEL_DIAMETER_MM         65.0f
#define ENCODER_PULLUP            1
#define ENCODER_COUNT_BOTH_EDGES  0

// ==== Motor direction calibration ====
#define MOTOR_R_INVERT 1
#define MOTOR_L_INVERT 0

// ==== PWM / driver ====
#define PWM_WRAP_8BIT       255
#define PWM_CLKDIV_DEFAULT  4.0f
#define PWM_WRAP            4095

// ==== Motor deadband & characterization ====
#define MOTOR_DEADBAND_PERCENT  8.0f
#define MOTOR_L_MIN_PWM        20.0f
#define MOTOR_L_MAX_PWM        36.0f
#define MOTOR_R_MIN_PWM        18.0f
#define MOTOR_R_MAX_PWM        35.0f

// ==== Speed & stall protection ====
#define SPEED_MAX_MM_S        350.0f
#define STALL_PWM_THRESHOLD     0.50f
#define STALL_WINDOW_MS       200
