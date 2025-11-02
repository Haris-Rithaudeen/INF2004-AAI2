#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "config.h"
#include "drivers/magnetometer.h"
#include "drivers/motor.h"
#include "fsm.h"

// FSM States
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_TURNING_90,
    STATE_TURNING_180,
    STATE_HOLD,
    STATE_COMPLETE
} turn_state_t;

// Turn control structure
typedef struct {
    turn_state_t state;
    float start_heading;
    float target_heading;
    float current_heading;
    float heading_error;
    uint32_t turn_start_time;
    uint32_t hold_start_time;
    bool turn_complete;
} turn_control_t;

static turn_control_t turn_ctrl = {0};
static uint32_t last_print_ms = 0;

static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// Normalize angle to 0-360 range
static float normalize_angle(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// Calculate shortest angular difference (-180 to +180)
static float angle_difference(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// Stop motors
static void stop_motors(void) {
    motor_stop();
}

// Turn in place STATIONARY (both wheels move opposite directions at same speed)
// Positive speed = turn right (clockwise), Negative speed = turn left (counter-clockwise)
static void turn_in_place(float speed) {
    // For stationary turning:
    // Right wheel goes one direction, left wheel goes opposite direction
    // This makes the robot spin around its center point
    
    if (speed > 0) {
        // Turn RIGHT (clockwise): right backward, left forward
        motor_set_signed(-speed, speed);
    } else {
        // Turn LEFT (counter-clockwise): right forward, left backward
        motor_set_signed(-speed, speed);  // speed is already negative
    }
}

// Initialize turn to a new target
static void start_turn(float target_degrees) {
    magnetometer_data_t data;
    
    // Get current heading (wait until we have a valid reading)
    int retries = 10;
    while (retries-- > 0) {
        if (magnetometer_read_data(&data)) {
            turn_ctrl.start_heading = data.heading;
            turn_ctrl.target_heading = normalize_angle(target_degrees);
            turn_ctrl.current_heading = data.heading;
            turn_ctrl.heading_error = angle_difference(turn_ctrl.target_heading, turn_ctrl.current_heading);
            turn_ctrl.turn_start_time = now_ms();
            turn_ctrl.turn_complete = false;
            
            printf("\n=== TURN START ===\n");
            printf("Start heading: %.1f°\n", turn_ctrl.start_heading);
            printf("Target heading: %.1f°\n", turn_ctrl.target_heading);
            printf("Initial error: %.1f°\n", turn_ctrl.heading_error);
            printf("Turn direction: %s\n", turn_ctrl.heading_error > 0 ? "RIGHT (CW)" : "LEFT (CCW)");
            return;
        }
        sleep_ms(10);
    }
    
    printf("ERROR: Failed to read magnetometer for turn start\n");
}

// Update turning control
static void update_turn(void) {
    magnetometer_data_t data;
    
    // Read current heading
    if (!magnetometer_read_data(&data)) {
        return;  // Skip this cycle if read fails
    }
    
    turn_ctrl.current_heading = data.heading;
    turn_ctrl.heading_error = angle_difference(turn_ctrl.target_heading, turn_ctrl.current_heading);
    
    uint32_t now = now_ms();
    
    // Check for timeout
    if (now - turn_ctrl.turn_start_time > TURN_TIMEOUT_MS) {
        printf("TIMEOUT: Turn took too long, stopping\n");
        stop_motors();
        turn_ctrl.state = STATE_COMPLETE;
        return;
    }
    
    // Check if we're within tolerance
    float abs_error = fabsf(turn_ctrl.heading_error);
    
    if (abs_error <= HEADING_SETTLE_TOLERANCE) {
        // We're very close - enter hold state to confirm
        if (turn_ctrl.state != STATE_HOLD) {
            turn_ctrl.state = STATE_HOLD;
            turn_ctrl.hold_start_time = now;
            stop_motors();
            printf("✓ Entering HOLD state (error: %.2f°)\n", turn_ctrl.heading_error);
        }
        
        // Check if we've held position long enough
        if (now - turn_ctrl.hold_start_time >= HOLD_TIME_MS) {
            turn_ctrl.turn_complete = true;
            stop_motors();
            
            float total_rotation = fabsf(angle_difference(turn_ctrl.current_heading, turn_ctrl.start_heading));
            
            printf("\n=== ✓ TURN COMPLETE ===\n");
            printf("Target: %.1f° | Final: %.1f° | Error: %.2f°\n", 
                   turn_ctrl.target_heading, turn_ctrl.current_heading, turn_ctrl.heading_error);
            printf("Total rotation: %.1f°\n", total_rotation);
            printf("Turn duration: %lu ms\n", now - turn_ctrl.turn_start_time);
            printf("Status: %s\n", abs_error <= HEADING_TOLERANCE ? "PASS (<5° error)" : "FAIL (>5° error)");
            
            turn_ctrl.state = STATE_COMPLETE;
        }
    } else {
        // We're not at target yet, continue turning
        if (turn_ctrl.state == STATE_HOLD) {
            // We fell out of tolerance during hold, restart turning
            printf("! Position drift detected (%.2f°), resuming turn\n", turn_ctrl.heading_error);
        }
        turn_ctrl.state = (fabsf(turn_ctrl.heading_error) > 90.0f) ? STATE_TURNING_180 : STATE_TURNING_90;
        
        // Calculate turn speed with smooth approach using PROPORTIONAL control
        float speed;
        if (abs_error > APPROACH_THRESHOLD) {
            // Far from target - use max speed
            speed = MAX_TURN_SPEED;
        } else {
            // Close to target - slow down proportionally
            // Linear interpolation between MIN and MAX based on error
            float ratio = abs_error / APPROACH_THRESHOLD;
            speed = MIN_TURN_SPEED + (MAX_TURN_SPEED - MIN_TURN_SPEED) * ratio;
        }
        
        // Apply direction (positive error = turn clockwise/right)
        if (turn_ctrl.heading_error > 0) {
            turn_in_place(speed);  // Turn right/clockwise
        } else {
            turn_in_place(-speed); // Turn left/counter-clockwise
        }
        
        // Print status
        if (now - last_print_ms >= PRINT_INTERVAL_MS) {
            printf("→ %.1f° | Target: %.1f° | Error: %+.2f° | Speed: %.1f%% | Dir: %s\n",
                   turn_ctrl.current_heading, 
                   turn_ctrl.target_heading, 
                   turn_ctrl.heading_error, 
                   speed,
                   turn_ctrl.heading_error > 0 ? "R" : "L");
            last_print_ms = now;
        }
    }
}

void fsm_init(void) {
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB serial
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║          PRECISE STATIONARY TURNING FSM                   ║\n");
    printf("║     Turn 90° and 180° with <5° error                      ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Configuration:\n");
    printf("  Min Turn Speed: %.1f%%\n", MIN_TURN_SPEED);
    printf("  Max Turn Speed: %.1f%%\n", MAX_TURN_SPEED);
    printf("  Approach Threshold: %.1f°\n", APPROACH_THRESHOLD);
    printf("  Settle Tolerance: %.1f°\n", HEADING_SETTLE_TOLERANCE);
    printf("  Hold Time: %d ms\n", HOLD_TIME_MS);
    printf("  Magnetometer Filter: %d samples\n\n", MAG_FILTER_SIZE);
    
    // Initialize magnetometer
    if (!magnetometer_init()) {
        printf("ERROR: Magnetometer initialization failed!\n");
        printf("Check I2C wiring: SDA=GP%d, SCL=GP%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
        while (1) {
            sleep_ms(1000);
        }
    }
    
    // Initialize motors
    motor_init_all();
    printf("✓ Motors initialized\n");
    
    // Wait for magnetometer to stabilize and fill filter buffer
    printf("Stabilizing magnetometer");
    for (int i = 0; i < MAG_FILTER_SIZE + 5; i++) {
        magnetometer_data_t data;
        magnetometer_read_data(&data);
        printf(".");
        sleep_ms(50);
    }
    printf(" ✓\n\n");
    
    turn_ctrl.state = STATE_INIT;
    printf("Starting turning sequence in 2 seconds...\n");
    sleep_ms(2000);
}

void fsm_step(void) {
    switch (turn_ctrl.state) {
        case STATE_INIT: {
            // Get initial heading
            magnetometer_data_t data;
            if (magnetometer_read_data(&data)) {
                printf("\nInitial heading: %.1f°\n", data.heading);
                turn_ctrl.state = STATE_IDLE;
            }
            break;
        }
        
        case STATE_IDLE: {
            // Start first turn (90 degrees from current position)
            magnetometer_data_t data;
            if (magnetometer_read_data(&data)) {
                float target = normalize_angle(data.heading + 90.0f);
                printf("\n╔════════════════════════════════════╗\n");
                printf("║       STARTING 90° TURN            ║\n");
                printf("╚════════════════════════════════════╝\n");
                start_turn(target);
                turn_ctrl.state = STATE_TURNING_90;
            }
            break;
        }
        
        case STATE_TURNING_90:
        case STATE_TURNING_180: {
            update_turn();
            break;
        }
        
        case STATE_HOLD: {
            // Continue holding and checking
            update_turn();
            break;
        }
        
        case STATE_COMPLETE: {
            // Check which turn just completed
            static int turn_count = 0;
            turn_count++;
            
            if (turn_count == 1) {
                // First turn (90°) complete, wait then start 180° turn
                printf("\n⏳ Waiting 3 seconds before 180° turn...\n");
                sleep_ms(3000);
                
                magnetometer_data_t data;
                if (magnetometer_read_data(&data)) {
                    float target = normalize_angle(data.heading + 180.0f);
                    printf("\n╔════════════════════════════════════╗\n");
                    printf("║       STARTING 180° TURN           ║\n");
                    printf("╚════════════════════════════════════╝\n");
                    start_turn(target);
                    turn_ctrl.state = STATE_TURNING_180;
                }
            } else {
                // Both turns complete
                printf("\n");
                printf("╔════════════════════════════════════════════════════════════╗\n");
                printf("║            ✓ ALL TURNS COMPLETE                           ║\n");
                printf("╚════════════════════════════════════════════════════════════╝\n");
                stop_motors();
                
                // Stay in this state (do nothing)
                while (1) {
                    sleep_ms(1000);
                }
            }
            break;
        }
    }
}