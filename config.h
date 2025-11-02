#pragma once
#include "pico/stdlib.h"

// ==== Motor pins ====
#define M1_A 8
#define M1_B 9
#define M2_A 10
#define M2_B 11

// ==== Encoder pins ====
#define LEFT_ENCODER_PIN   16
#define RIGHT_ENCODER_PIN  26

// ==== Direction calibration ====
#define MOTOR_R_INVERT 1
#define MOTOR_L_INVERT 0

// ==== Button pins ====
#define BUTTON_DIR    21
#define BUTTON_SPD    20

// ==== PWM config ====
#define PWM_WRAP_8BIT 255
#define PWM_CLKDIV_DEFAULT 4.0f

// ==== App config ====
#define DEBOUNCE_MS 200
#define DISPLAY_INTERVAL_MS 3000

// ==== IR Sensor 1: Line Follower ====
#define IR_LINE_ADC_GPIO              28
#define IR_LINE_ADC_CHANNEL           2
#define IR_LINE_DIGITAL_GPIO          7

// ==== IR Sensor 2: Barcode Scanner ====
#define IR_BARCODE_ADC_GPIO           27
#define IR_BARCODE_ADC_CHANNEL        1
#define IR_BARCODE_DIGITAL_GPIO       6

// ==== Ultrasonic Sensor ====
#define ULTRASONIC_TRIG_GPIO          5
#define ULTRASONIC_ECHO_GPIO          4

// ==== I2C Magnetometer/Accelerometer (LSM303) ====
#define I2C_PORT                i2c1
#define I2C_SDA_PIN             2
#define I2C_SCL_PIN             3
#define I2C_BAUDRATE            100000
#define LSM303_ACC_ADDR         0x19
#define LSM303_MAG_ADDR         0x1E

// ===== Control & hardware config =====
#define CTRL_HZ                 100
#define CONTROL_DT_MS           (1000/CTRL_HZ)
#define ENCODER_CPR             20.0f
#define WHEEL_DIAMETER_MM       65.0f
#define ENCODER_PULLUP          1
#define PWM_WRAP                4095
#define STALL_PWM_THRESHOLD     0.50f
#define STALL_WINDOW_MS         200
#define ENCODER_COUNT_BOTH_EDGES 0

// ===== Line Follower IR config =====
#define IR_LINE_WHITE_HIGH           0       // 0 = white reads low, black reads high
#define IR_LINE_THRESHOLD            ((IR_LINE_EDGE_LOW_LIMIT + IR_LINE_EDGE_HIGH_LIMIT) / 2.0f)    // Middle of your range (1500+2300)/2
#define IR_LINE_PRINT_INTERVAL_MS    500
#define IR_LINE_EDGE_LOW_LIMIT       1000
#define IR_LINE_EDGE_HIGH_LIMIT      2100

// ===== Line Following PID Tuning =====
// Smoother tuning for circular tracks
#define LINE_FOLLOW_KP          0.0045f    // Proportional - responsive but smooth
#define LINE_FOLLOW_KI          0.0001f   // Very low integral - prevents overshoot on curves
#define LINE_FOLLOW_KD          0.030f    // Higher derivative - smoother on curves
#define LINE_CORRECTION_MAX     10.0f     // Max steering correction

// Smoothing for sensor readings (helps with circular tracks)
#define IR_SENSOR_FILTER_ALPHA  0.15f      // Lower = smoother (0.1-0.5 range)


// ===== Barcode Scanner IR config =====
#define IR_BARCODE_WHITE_HIGH        0
#define IR_BARCODE_THRESHOLD         2000
#define IR_BARCODE_SAMPLE_RATE_US    1000
#define BARCODE_MAX_BARS             50
#define BARCODE_DELIMITER_LIMIT      2       // Number of '*' delimiters to detect end

// ===== Barcode Turn Configuration =====
#define BARCODE_TURN_SPEED           20.0f   // Speed for barcode-triggered turns
#define BARCODE_TURN_TIMEOUT_MS      8000    // Timeout for turns

// ===== Ultrasonic Sensor config =====
#define ULTRASONIC_SPEED_CM_PER_US      0.0343f
#define ULTRASONIC_MIN_DISTANCE_CM      2.0f
#define ULTRASONIC_MAX_DISTANCE_CM      400.0f
#define ULTRASONIC_ECHO_TIMEOUT_US      25000
#define ULTRASONIC_PRINT_INTERVAL_MS    500

// ===== Magnetometer config =====
#define MAG_PRINT_INTERVAL_MS           500
#define MAG_FILTER_SIZE                 10

//==== Tuning Parameters for Precise Turning =====
// Accuracy thresholds
#define HEADING_TOLERANCE       5.0f        // Final acceptable error (don't change - this is your requirement)
#define HEADING_SETTLE_TOLERANCE 2.0f       // Must be within this to start holding (lower = more precise, but may oscillate)

// Speed control
#define MIN_TURN_SPEED          15.0f       // Minimum PWM % - increase if robot doesn't move smoothly at low speeds
#define MAX_TURN_SPEED          25.0f       // Maximum PWM % - decrease if overshooting, increase if too slow
#define APPROACH_THRESHOLD      20.0f       // Start slowing down when this many degrees from target (higher = smoother but slower)

// Timing
#define HOLD_TIME_MS            300         // How long to hold position before confirming (increase if drifting after "complete")
#define TURN_TIMEOUT_MS         4000       // Safety timeout (15 seconds)
#define PRINT_INTERVAL_MS       100          // Status print rate

// ===== Motor Deadband Compensation =====
#define MOTOR_DEADBAND_PERCENT     8.0f

// ===== Motor Characterization (PER WHEEL) =====
#define MOTOR_L_MIN_PWM      18.0f
#define MOTOR_L_MAX_PWM      35.0f

#define MOTOR_R_MIN_PWM      18.0f
#define MOTOR_R_MAX_PWM      35.0f

// ===== PID Gains (PER WHEEL) =====
#define SPEED_MAX_MM_S             350.0f

#define PID_L_KP             0.3f
#define PID_L_KI             0.01f
#define PID_L_KD             0.0f

#define PID_R_KP             0.35f
#define PID_R_KI             0.01f
#define PID_R_KD             0.0f

// PID output limits (same for both)
#define PID_OUT_MIN         (-40.0f)
#define PID_OUT_MAX          (40.0f)
#define PID_INTEG_MIN       (-5.0f)
#define PID_INTEG_MAX        (5.0f)

// ===== IMU Heading Correction =====
#define HEADING_KP                2.5f
#define HEADING_KI                0.4f
#define HEADING_KD                0.5f
#define HEADING_DEADZONE          1.0f
#define MAX_HEADING_CORRECTION   30.0f