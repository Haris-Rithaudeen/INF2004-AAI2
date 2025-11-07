#pragma once
// ==== LSM303 device addresses ====
#define LSM303_ACC_ADDR  0x19
#define LSM303_MAG_ADDR  0x1E

// ==== Line follower IR ====
#define IR_LINE_WHITE_HIGH        0   // 0: white low, black high
#define IR_LINE_EDGE_LOW_LIMIT  1400
#define IR_LINE_EDGE_HIGH_LIMIT 2100
#define IR_LINE_THRESHOLD ((IR_LINE_EDGE_LOW_LIMIT + IR_LINE_EDGE_HIGH_LIMIT) / 2.0f)
#define IR_LINE_PRINT_INTERVAL_MS  500

// ==== Barcode IR ====
#define IR_BARCODE_WHITE_HIGH     0
#define IR_BARCODE_THRESHOLD   2000
#define IR_BARCODE_SAMPLE_RATE_US 1000
#define BARCODE_MAX_BARS          50
#define BARCODE_DELIMITER_LIMIT    2

// ==== Ultrasonic ====
#define ULTRASONIC_SPEED_CM_PER_US   0.0343f
#define ULTRASONIC_MIN_DISTANCE_CM   2.0f
#define ULTRASONIC_MAX_DISTANCE_CM   400.0f
#define ULTRASONIC_ECHO_TIMEOUT_US   25000
#define ULTRASONIC_PRINT_INTERVAL_MS  500

// ==== Magnetometer / heading ====
#define MAG_PRINT_INTERVAL_MS   500
#define MAG_FILTER_SIZE         10
