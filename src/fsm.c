#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "config.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/magnetometer.h"
#include "control/pid.h"
#include "networking/wifi_mqtt.h"
#include "fsm.h"

// ---- Unified GPIO ISR for buttons + encoders ----
static volatile bool g_toggle_dir_req = false;
static volatile bool g_change_speed_req = false;
static volatile uint32_t g_last_dir_ms = 0;
static volatile uint32_t g_last_spd_ms = 0;

static inline uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static void gpio_isr_unified(uint gpio, uint32_t events) {
    encoder_on_gpio_irq(gpio, events);
    uint32_t t = now_ms();
    if (gpio == BUTTON_DIR && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_dir_ms) > DEBOUNCE_MS) { g_toggle_dir_req = true; g_last_dir_ms = t; }
    }
    if (gpio == BUTTON_SPD && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_spd_ms) > DEBOUNCE_MS) { g_change_speed_req = true; g_last_spd_ms = t; }
    }
}

// ---- App state ----
static bool forward_mode = true;
static uint speed_pct = 70;
static uint32_t last_print_ms = 0;
static float target_heading = 0.0f;
static bool heading_locked = false;

// MQTT telemetry timing
static uint32_t last_telemetry_ms = 0;
#define TELEMETRY_INTERVAL_MS 100  // Publish telemetry every 100ms

// Startup state machine
typedef enum {
    STATE_IMU_WARMUP,      // Filling filter buffer
    STATE_MOTOR_RAMPUP,    // Ramping up motors before locking heading
    STATE_HEADING_LOCK,    // Lock heading after motors stabilize
    STATE_RUNNING          // Normal operation
} startup_state_t;

static startup_state_t startup_state = STATE_IMU_WARMUP;
static uint32_t state_timer = 0;
static uint8_t imu_samples_collected = 0;
static float ramp_multiplier = 0.0f;

#define IMU_WARMUP_SAMPLES 16       // Fill filter with 16 samples
#define MOTOR_RAMPUP_MS    1000     // 1 second motor ramp-up
#define HEADING_LOCK_DELAY_MS 500   // Wait 500ms after ramp before locking heading

// PID controllers
static pid_t pid_r, pid_l, pid_heading;

// cached for velocity estimate
static int32_t prev_l_ticks = 0, prev_r_ticks = 0;

static inline float dt_s(void) { return CONTROL_DT_MS / 1000.0f; }

// Normalize angle difference to [-180, +180]
static float normalize_angle_error(float error) {
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

void fsm_init(void) {
    stdio_init_all();
    motor_init_all();
    encoder_init();
    wifi_mqtt_init();
    wifi_mqtt_connect();
    mqtt_connect_broker();

    // Initialize magnetometer
    if (!magnetometer_init()) {
        printf("ERROR: Magnetometer initialization failed!\n");
        printf("IMU-assisted straight motion will NOT work.\n");
    } else {
        printf("✓ Magnetometer initialized with %d-sample moving average filter\n", MAG_FILTER_SIZE);
    }

    // Buttons
    gpio_init(BUTTON_DIR); gpio_set_dir(BUTTON_DIR, GPIO_IN); gpio_pull_up(BUTTON_DIR);
    gpio_init(BUTTON_SPD); gpio_set_dir(BUTTON_SPD, GPIO_IN); gpio_pull_up(BUTTON_SPD);

    // Register unified ISR
#if ENCODER_COUNT_BOTH_EDGES
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
#else
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE;
#endif
    gpio_set_irq_enabled_with_callback(BUTTON_DIR, GPIO_IRQ_EDGE_FALL, true, &gpio_isr_unified);
    gpio_set_irq_enabled(BUTTON_SPD, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(LEFT_ENCODER_PIN,  enc_edge, true);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, enc_edge, true);

    // PID setup for velocity control
    pid_init(&pid_r, PID_R_KP, PID_R_KI, PID_R_KD, dt_s(),
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);
    pid_init(&pid_l, PID_L_KP, PID_L_KI, PID_L_KD, dt_s(),
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);

    // PID setup for heading correction
    pid_init(&pid_heading, HEADING_KP, HEADING_KI, HEADING_KD, dt_s(),
             -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION, -50.0f, 50.0f);

    // Motors OFF during startup
    motor_set_signed(0, 0);

    // seed tick baselines
    prev_l_ticks = (int32_t)encoder_left_count();
    prev_r_ticks = (int32_t)encoder_right_count();

    printf("\n=== FSM with IMU-Assisted Straight Motion ===\n");
    printf("CPR=%.0f | wheel=%.1fmm | mm/tick=%.2f | dt=%.3fs\n",
           ENCODER_CPR, WHEEL_DIAMETER_MM, encoder_mm_per_tick(), dt_s());
    printf("GP21 (DIR): Toggle forward/backward & lock current heading\n");
    printf("GP20 (SPD): Randomize speed 40-100%%\n\n");
    
    printf("⏳ Stage 1: Warming up IMU filter...\n");
    
    startup_state = STATE_IMU_WARMUP;
    state_timer = now_ms();
    imu_samples_collected = 0;
    heading_locked = false;
    ramp_multiplier = 0.0f;
    last_telemetry_ms = 0;
}

// helper: UI % -> target wheel speed (mm/s), signed by direction
static float target_mm_s_from_ui(void) {
    float sign = forward_mode ? +1.0f : -1.0f;
    return sign * (SPEED_MAX_MM_S * (speed_pct / 100.0f)) * ramp_multiplier;
}

void fsm_step(void) {
    uint32_t t = now_ms();
    
    // ========== STARTUP STATE MACHINE ==========
    switch (startup_state) {
        case STATE_IMU_WARMUP: {
            // Collect IMU samples to fill the moving average filter
            magnetometer_data_t mag_data;
            if (magnetometer_read_data(&mag_data)) {
                imu_samples_collected++;
                
                if (imu_samples_collected >= IMU_WARMUP_SAMPLES) {
                    printf("✓ IMU warmup complete (%d samples)\n", imu_samples_collected);
                    printf("⏳ Stage 2: Ramping up motors...\n");
                    
                    startup_state = STATE_MOTOR_RAMPUP;
                    state_timer = t;
                    ramp_multiplier = 0.0f;
                }
            }
            
            // Keep motors stopped during warmup
            motor_set_signed(0, 0);
            sleep_ms(CONTROL_DT_MS);
            return;
        }
        
        case STATE_MOTOR_RAMPUP: {
            // Gradually ramp up motor speed over MOTOR_RAMPUP_MS
            uint32_t elapsed = t - state_timer;
            
            if (elapsed >= MOTOR_RAMPUP_MS) {
                ramp_multiplier = 1.0f;
                printf("✓ Motors ramped up to full speed\n");
                printf("⏳ Stage 3: Stabilizing before heading lock...\n");
                
                startup_state = STATE_HEADING_LOCK;
                state_timer = t;
            } else {
                // Linear ramp from 0.0 to 1.0
                ramp_multiplier = (float)elapsed / (float)MOTOR_RAMPUP_MS;
            }
            
            // Continue to motor control below (with ramped target)
            break;
        }
        
        case STATE_HEADING_LOCK: {
            // Wait a bit more for motors to stabilize, then lock heading
            uint32_t elapsed = t - state_timer;
            
            if (elapsed >= HEADING_LOCK_DELAY_MS) {
                magnetometer_data_t mag_data;
                if (magnetometer_read_data(&mag_data)) {
                    target_heading = mag_data.heading;
                    heading_locked = true;
                    
                    printf("✓ Heading locked: %.1f°\n", target_heading);
                    printf("✓ System ready - full operation mode\n\n");
                    
                    startup_state = STATE_RUNNING;
                }
            }
            
            // Continue to motor control below (no heading correction yet)
            break;
        }
        
        case STATE_RUNNING:
            // Normal operation - handled below
            ramp_multiplier = 1.0f;
            break;
    }
    
    // ========== BUTTON HANDLING ==========
    if (g_toggle_dir_req) {
        g_toggle_dir_req = false;
        forward_mode = !forward_mode;
        
        // Reset PIDs to avoid kick
        pid_reset(&pid_r); 
        pid_reset(&pid_l);
        pid_reset(&pid_heading);
        
        // Lock new heading when direction changes
        magnetometer_data_t mag_data;
        if (magnetometer_read_data(&mag_data)) {
            target_heading = mag_data.heading;
            heading_locked = true;
            printf("Direction: %s | New heading locked: %.1f°\n", 
                   forward_mode ? "FORWARD" : "BACKWARD", target_heading);
        }
    }
    
    if (g_change_speed_req) {
        g_change_speed_req = false;
        speed_pct = 40 + (rand() % 61);
        printf("Speed setpoint: %u%%\n", speed_pct);
    }

    // ========== READ IMU FOR HEADING CORRECTION ==========
    magnetometer_data_t mag_data;
    float heading_correction = 0.0f;
    bool imu_ok = magnetometer_read_data(&mag_data);
    
    // Only apply heading correction when fully running and moving
    if (startup_state == STATE_RUNNING && imu_ok && heading_locked && speed_pct > 20) {
        float heading_error = normalize_angle_error(target_heading - mag_data.heading);
        
        if (fabsf(heading_error) > HEADING_DEADZONE) {
            heading_correction = pid_update(&pid_heading, 0.0f, heading_error);
        } else {
            // Within deadzone - reset integral to prevent windup
            pid_reset(&pid_heading);
        }
    }

    // ========== MEASURE WHEEL SPEEDS ==========
    int32_t cur_l = (int32_t)encoder_left_count();
    int32_t cur_r = (int32_t)encoder_right_count();
    int32_t dlt   = cur_l - prev_l_ticks;
    int32_t drt   = cur_r - prev_r_ticks;
    prev_l_ticks = cur_l;
    prev_r_ticks = cur_r;

    float mm_per_tick = encoder_mm_per_tick();
    float meas_l_mm_s = (dlt * mm_per_tick) / dt_s();
    float meas_r_mm_s = (drt * mm_per_tick) / dt_s();

    // ========== PID VELOCITY CONTROL ==========
    float tgt_mm_s = target_mm_s_from_ui();  // Already includes ramp_multiplier
    float u_r_base = pid_update(&pid_r, tgt_mm_s, meas_r_mm_s);
    float u_l_base = pid_update(&pid_l, tgt_mm_s, meas_l_mm_s);

    // ========== APPLY HEADING CORRECTION ==========
    float u_r = u_r_base - heading_correction;
    float u_l = u_l_base + heading_correction;

    // Clamp to [-100, 100]
    if (u_r > 100.0f) u_r = 100.0f;
    if (u_r < -100.0f) u_r = -100.0f;
    if (u_l > 100.0f) u_l = 100.0f;
    if (u_l < -100.0f) u_l = -100.0f;

    // ========== DRIVE MOTORS ==========
    motor_set_signed(u_r, u_l);

    // ========== PUBLISH MQTT TELEMETRY ==========
    if (mqtt_is_connected() && (t - last_telemetry_ms >= TELEMETRY_INTERVAL_MS)) {
        // Publish motor telemetry (convert mm/s to cm/s, mm to cm)
        float left_speed_cm_s = meas_l_mm_s / 10.0f;
        float right_speed_cm_s = meas_r_mm_s / 10.0f;
        float left_dist_cm = (cur_l * mm_per_tick) / 10.0f;
        float right_dist_cm = (cur_r * mm_per_tick) / 10.0f;
        
        mqtt_publish_motors(left_speed_cm_s, right_speed_cm_s, 
                           left_dist_cm, right_dist_cm);
        
        // Publish IMU data if available
        if (imu_ok) {
            mqtt_publish_imu(mag_data.heading,
                           mag_data.x, mag_data.y, mag_data.z,
                           0, 0, 0);  // No accelerometer data in this system
        }
        
        // Publish PID data (using right wheel as reference)
        float pid_error = tgt_mm_s - meas_r_mm_s;
        mqtt_publish_pid(PID_R_KP, PID_R_KI, PID_R_KD, pid_error, u_r_base);
        
        // Publish state
        const char* state_str;
        switch (startup_state) {
            case STATE_IMU_WARMUP:
                state_str = "IMU_WARMUP";
                break;
            case STATE_MOTOR_RAMPUP:
                state_str = "MOTOR_RAMPUP";
                break;
            case STATE_HEADING_LOCK:
                state_str = "HEADING_LOCK";
                break;
            case STATE_RUNNING:
                state_str = forward_mode ? "FORWARD" : "BACKWARD";
                break;
            default:
                state_str = "UNKNOWN";
        }
        mqtt_publish_state(state_str, "");
        
        last_telemetry_ms = t;
    }

    // ========== STATUS PRINT ==========
    if (t - last_print_ms >= DISPLAY_INTERVAL_MS) {
        printf("========== Status ==========\n");
        
        if (startup_state != STATE_RUNNING) {
            printf("Startup: Stage %d | Ramp: %.1f%%\n", 
                   (int)startup_state, ramp_multiplier * 100.0f);
        } else {
            printf("Mode: %s | Speed: %u%% | Target: %.0f mm/s\n", 
                   forward_mode ? "FORWARD" : "BACKWARD", speed_pct, tgt_mm_s);
        }
        
        printf("Velocity - L: %.0f mm/s | R: %.0f mm/s\n", meas_l_mm_s, meas_r_mm_s);
        
        if (imu_ok) {
            float heading_error = normalize_angle_error(target_heading - mag_data.heading);
            printf("IMU - Current: %.1f° | Target: %.1f° | Error: %.1f°\n", 
                   mag_data.heading, target_heading, heading_error);
            printf("Heading Correction: %.1f%% | Motor Duty - L: %.1f%% | R: %.1f%%\n",
                   heading_correction, u_l, u_r);
            
            if (heading_locked) {
                printf("Status: %s\n", 
                       fabsf(heading_error) < HEADING_DEADZONE ? "✓ ON TRACK" : "⚠ CORRECTING DRIFT");
            } else {
                printf("Status: ⏳ Heading not locked yet\n");
            }
        } else {
            printf("IMU - ✗ READ FAILED | Motor Duty - L: %.1f%% | R: %.1f%%\n", u_l, u_r);
        }
        
        // Show MQTT connection status
        printf("MQTT: %s\n", mqtt_is_connected() ? "✓ Connected" : "✗ Disconnected");
        printf("============================\n\n");
        
        last_print_ms = t;
        
        encoder_reset_counts();
        prev_l_ticks = prev_r_ticks = 0;
    }
}