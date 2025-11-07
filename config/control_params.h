#pragma once
// ==== Control loop rates ====
#define CTRL_HZ           100
#define CONTROL_DT_MS     (1000/CTRL_HZ)

// ==== App / UI timings (general) ====
#define DEBOUNCE_MS          200
#define DISPLAY_INTERVAL_MS 3000

// ==== Line following PID ====
#define LINE_FOLLOW_KP        0.0045f
#define LINE_FOLLOW_KI        0.0001f
#define LINE_FOLLOW_KD        0.030f
#define LINE_CORRECTION_MAX   10.0f

// Sensor smoothing used by controller
#define IR_SENSOR_FILTER_ALPHA 0.15f

// ==== Per-wheel speed PID ====
#define PID_L_KP   0.3f
#define PID_L_KI   0.01f
#define PID_L_KD   0.0f

#define PID_R_KP   0.25f
#define PID_R_KI   0.01f
#define PID_R_KD   0.0f

#define PID_OUT_MIN    (-40.0f)
#define PID_OUT_MAX      40.0f
#define PID_INTEG_MIN   (-5.0f)
#define PID_INTEG_MAX     5.0f

// ==== Heading control (IMU) ====
#define HEADING_KP                3.0f
#define HEADING_KI                0.4f
#define HEADING_KD                0.5f
#define HEADING_DEADZONE          1.0f
#define MAX_HEADING_CORRECTION   40.0f

// ==== Precise turning (barcode-triggered, etc.) ====
#define HEADING_TOLERANCE          5.0f
#define HEADING_SETTLE_TOLERANCE   2.0f
#define MIN_TURN_SPEED            15.0f
#define MAX_TURN_SPEED            25.0f
#define APPROACH_THRESHOLD        20.0f
#define HOLD_TIME_MS             300
#define TURN_TIMEOUT_MS         4000
#define PRINT_INTERVAL_MS        100
