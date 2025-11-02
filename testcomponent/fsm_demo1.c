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
#include "fsm.h"

// ======================================================================
// FSM_DEMO1: Per-wheel PID + IMU heading correction (SIGN FIXED)
// Purpose: Move in a perfectly straight line using magnetometer feedback
// ======================================================================

// ---- GPIO ISR ----
static volatile bool g_toggle_dir_req = false;
static volatile bool g_change_speed_req = false;
static volatile uint32_t g_last_dir_ms = 0;
static volatile uint32_t g_last_spd_ms = 0;

static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void gpio_isr_unified(uint gpio, uint32_t events) {
    encoder_on_gpio_irq(gpio, events);
    
    uint32_t t = now_ms();
    if (gpio == BUTTON_DIR && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_dir_ms) > DEBOUNCE_MS) { 
            g_toggle_dir_req = true; 
            g_last_dir_ms = t; 
        }
    }
    if (gpio == BUTTON_SPD && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_spd_ms) > DEBOUNCE_MS) { 
            g_change_speed_req = true; 
            g_last_spd_ms = t; 
        }
    }
}

// ---- State ----
static bool forward = true;
static uint speed_pct = 50;
static uint32_t last_print_ms = 0;
static uint32_t last_control_ms = 0;

// Per-wheel PIDs + Heading correction PID
static pid_t pid_r, pid_l, pid_heading;

#define VEL_WINDOW_MS 150
static uint32_t vel_window_start = 0;
static uint32_t vel_l_start = 0;
static uint32_t vel_r_start = 0;
static float vel_l = 0.0f;
static float vel_r = 0.0f;

static float total_dist_l_mm = 0.0f;
static float total_dist_r_mm = 0.0f;

// IMU/Heading tracking
static float target_heading = 0.0f;
static bool heading_locked = false;
static bool imu_ready = false;

// Startup state machine for smooth initialization
typedef enum {
    STATE_IMU_WARMUP,      // Fill magnetometer filter
    STATE_HEADING_LOCK,    // Lock initial heading
    STATE_RUNNING          // Normal operation
} startup_state_t;

static startup_state_t startup_state = STATE_IMU_WARMUP;
static uint8_t imu_samples = 0;
#define IMU_WARMUP_SAMPLES 10

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

    // Initialize magnetometer
    if (!magnetometer_init()) {
        printf("⚠️  WARNING: Magnetometer init failed! IMU correction disabled.\n");
        imu_ready = false;
    } else {
        printf("✓ Magnetometer initialized (%d-sample moving average)\n", MAG_FILTER_SIZE);
        imu_ready = true;
    }

    gpio_init(BUTTON_DIR); gpio_set_dir(BUTTON_DIR, GPIO_IN); gpio_pull_up(BUTTON_DIR);
    gpio_init(BUTTON_SPD); gpio_set_dir(BUTTON_SPD, GPIO_IN); gpio_pull_up(BUTTON_SPD);

#if ENCODER_COUNT_BOTH_EDGES
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
#else
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE;
#endif
    gpio_set_irq_enabled_with_callback(BUTTON_DIR, GPIO_IRQ_EDGE_FALL, true, &gpio_isr_unified);
    gpio_set_irq_enabled(BUTTON_SPD, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(LEFT_ENCODER_PIN,  enc_edge, true);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, enc_edge, true);

    float dt = 0.01f;
    
    // Initialize LEFT wheel PID
    pid_init(&pid_l, PID_L_KP, PID_L_KI, PID_L_KD, dt,
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);
    
    // Initialize RIGHT wheel PID
    pid_init(&pid_r, PID_R_KP, PID_R_KI, PID_R_KD, dt,
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);

    // Initialize HEADING correction PID
    pid_init(&pid_heading, HEADING_KP, HEADING_KI, HEADING_KD, dt,
             -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION, -20.0f, 20.0f);

    motor_set_signed(0, 0);

    vel_l_start = encoder_left_count();
    vel_r_start = encoder_right_count();
    vel_window_start = now_ms();
    last_control_ms = vel_window_start;

    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║  FSM_DEMO1: PER-WHEEL PID + IMU HEADING CORRECTION (v3)     ║\n");
    printf("║  Purpose: Move in a PERFECTLY STRAIGHT LINE                 ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    printf("Encoder: CPR=%.0f | Wheel=%.1fmm | Resolution=%.2fmm\n",
           ENCODER_CPR, WHEEL_DIAMETER_MM, encoder_mm_per_tick());
    printf("Velocity window: %dms | Deadband comp: %.0f%%\n\n", 
           VEL_WINDOW_MS, MOTOR_DEADBAND_PERCENT);
    
    printf("┌─ LEFT MOTOR ────────────────────────────────────────────┐\n");
    printf("│ FF: %.1f%% to %.1f%% | PID: Kp=%.2f Ki=%.3f Kd=%.3f   │\n", 
           MOTOR_L_MIN_PWM, MOTOR_L_MAX_PWM, PID_L_KP, PID_L_KI, PID_L_KD);
    printf("└─────────────────────────────────────────────────────────┘\n\n");
    
    printf("┌─ RIGHT MOTOR ───────────────────────────────────────────┐\n");
    printf("│ FF: %.1f%% to %.1f%% | PID: Kp=%.2f Ki=%.3f Kd=%.3f   │\n", 
           MOTOR_R_MIN_PWM, MOTOR_R_MAX_PWM, PID_R_KP, PID_R_KI, PID_R_KD);
    printf("└─────────────────────────────────────────────────────────┘\n\n");
    
    printf("┌─ HEADING CORRECTION ────────────────────────────────────┐\n");
    printf("│ Kp=%.2f Ki=%.2f Kd=%.2f | Max=±%.0fmm/s | Dead=%.1f°│\n", 
           HEADING_KP, HEADING_KI, HEADING_KD, MAX_HEADING_CORRECTION, HEADING_DEADZONE);
    printf("│ SIGN CONVENTION: +error = drift left, -error = drift right│\n");
    printf("└─────────────────────────────────────────────────────────┘\n\n");
    
    if (imu_ready) {
        printf("⏳ Warming up IMU filter (%d samples)...\n\n", IMU_WARMUP_SAMPLES);
    } else {
        printf("⚠️  Running in FALLBACK mode (no IMU correction)\n\n");
        startup_state = STATE_RUNNING;
        heading_locked = false;
    }
    
    printf("GP21=dir (lock heading) | GP20=speed\n\n");
}

static float target_mm_s_from_ui(void) {
    float sign = forward ? +1.0f : -1.0f;
    return sign * (SPEED_MAX_MM_S * (speed_pct / 100.0f));
}

// Feedforward helper with deadband
static inline float ff_from_speed(float target_mm_s, float min_pwm, float max_pwm) {
    float s = target_mm_s;
    float sign = (s >= 0.0f) ? 1.0f : -1.0f;
    float mag = fabsf(s);

    float base = (SPEED_MAX_MM_S > 1.0f) ? (mag / SPEED_MAX_MM_S) : 0.0f;
    if (base < 0.0f) base = 0.0f;
    if (base > 1.0f) base = 1.0f;

    float pwm = min_pwm + (max_pwm - min_pwm) * base;

    float fade = 1.0f - base;
    float add  = (MOTOR_DEADBAND_PERCENT * 0.4f) * fade;
    pwm += add;

    return sign * pwm;
}

static float feedforward_left(float target_mm_s) {
    float pwm = ff_from_speed(target_mm_s, MOTOR_L_MIN_PWM, MOTOR_L_MAX_PWM);
    float limit = (pwm >= 0.0f) ? MOTOR_L_MAX_PWM : -MOTOR_L_MAX_PWM;
    if (pwm >  limit) pwm =  limit;
    if (pwm < -limit) pwm = -limit;
    return pwm;
}

static float feedforward_right(float target_mm_s) {
    float pwm = ff_from_speed(target_mm_s, MOTOR_R_MIN_PWM, MOTOR_R_MAX_PWM);
    float limit = (pwm >= 0.0f) ? MOTOR_R_MAX_PWM : -MOTOR_R_MAX_PWM;
    if (pwm >  limit) pwm =  limit;
    if (pwm < -limit) pwm = -limit;
    return pwm;
}

void fsm_step(void) {
    uint32_t now = now_ms();
    
    uint32_t dt_ms = now - last_control_ms;
    if (dt_ms < 5) return;
    last_control_ms = now;
    float dt_s = dt_ms / 1000.0f;
    
    // ========== STARTUP STATE MACHINE ==========
    if (imu_ready && startup_state != STATE_RUNNING) {
        magnetometer_data_t mag_data;
        
        switch (startup_state) {
            case STATE_IMU_WARMUP:
                if (magnetometer_read_data(&mag_data)) {
                    imu_samples++;
                    if (imu_samples >= IMU_WARMUP_SAMPLES) {
                        printf("✓ IMU filter ready\n");
                        printf("⏳ Locking initial heading...\n");
                        startup_state = STATE_HEADING_LOCK;
                    }
                }
                motor_set_signed(0, 0);
                sleep_ms(50);
                return;
                
            case STATE_HEADING_LOCK:
                if (magnetometer_read_data(&mag_data)) {
                    target_heading = mag_data.heading;
                    heading_locked = true;
                    startup_state = STATE_RUNNING;
                    printf("✓ Heading locked: %.1f° | System ready!\n\n", target_heading);
                }
                break;
                
            case STATE_RUNNING:
                break;
        }
    }
    
    // ========== Buttons ==========
    if (g_toggle_dir_req) {
        g_toggle_dir_req = false;
        forward = !forward;
        pid_reset(&pid_r); 
        pid_reset(&pid_l);
        pid_reset(&pid_heading);
        vel_l_start = encoder_left_count();
        vel_r_start = encoder_right_count();
        vel_window_start = now;
        vel_l = vel_r = 0.0f;
        
        if (imu_ready) {
            magnetometer_data_t mag_data;
            if (magnetometer_read_data(&mag_data)) {
                target_heading = mag_data.heading;
                heading_locked = true;
                printf("\n>>> DIR: %s | New heading: %.1f° <<<\n\n", 
                       forward ? "FWD" : "REV", target_heading);
            }
        } else {
            printf("\n>>> DIRECTION: %s <<<\n\n", forward ? "FWD" : "REV");
        }
    }
    
    if (g_change_speed_req) {
        g_change_speed_req = false;
        speed_pct = 40 + (rand() % 61);
        printf("\n>>> SPEED: %u%% = %.1f mm/s <<<\n\n", 
               speed_pct, SPEED_MAX_MM_S * speed_pct / 100.0f);
    }

    // ========== Update Velocity ==========
    uint32_t window_elapsed = now - vel_window_start;
    
    if (window_elapsed >= VEL_WINDOW_MS) {
        uint32_t cur_l = encoder_left_count();
        uint32_t cur_r = encoder_right_count();
        
        int32_t ticks_l = (int32_t)(cur_l - vel_l_start);
        int32_t ticks_r = (int32_t)(cur_r - vel_r_start);
        
        float actual_window_s = window_elapsed / 1000.0f;
        float mm_per_tick = encoder_mm_per_tick();
        
        vel_l = (ticks_l * mm_per_tick) / actual_window_s;
        vel_r = (ticks_r * mm_per_tick) / actual_window_s;

        // EMA smoothing
        static bool vel_ema_init = false;
        static float vel_l_ema = 0.0f, vel_r_ema = 0.0f;
        const float alpha_vel = 0.40f;

        if (!vel_ema_init) {
            vel_l_ema = vel_l;
            vel_r_ema = vel_r;
            vel_ema_init = true;
        } else {
            vel_l_ema = alpha_vel * vel_l + (1.0f - alpha_vel) * vel_l_ema;
            vel_r_ema = alpha_vel * vel_r + (1.0f - alpha_vel) * vel_r_ema;
        }
        vel_l = vel_l_ema;
        vel_r = vel_r_ema;
        
        total_dist_l_mm += ticks_l * mm_per_tick;
        total_dist_r_mm += ticks_r * mm_per_tick;
        
        vel_l_start = cur_l;
        vel_r_start = cur_r;
        vel_window_start = now;
    }

        // ========== Read IMU for Heading Correction ==========
    float heading_correction_mm_s = 0.0f;
    float current_heading = 0.0f;
    float heading_error = 0.0f;
    bool imu_read_ok = false;
    
    if (imu_ready && heading_locked && startup_state == STATE_RUNNING) {
        magnetometer_data_t mag_data;
        if (magnetometer_read_data(&mag_data)) {
            current_heading = mag_data.heading;
            heading_error = normalize_angle_error(target_heading - current_heading);
            imu_read_ok = true;
            
            // 🔥 NEW: Low-pass filter on heading error to smooth out jitter
            static float filtered_heading_error = 0.0f;
            static bool filter_init = false;
            const float alpha_heading = 0.3f;  // Lower = smoother (0.0 to 1.0)
            
            if (!filter_init) {
                filtered_heading_error = heading_error;
                filter_init = true;
            } else {
                filtered_heading_error = alpha_heading * heading_error + 
                                        (1.0f - alpha_heading) * filtered_heading_error;
            }
            
            // Use filtered error for PID
            float smoothed_error = filtered_heading_error;
            
            // Only apply correction outside deadzone and when moving
            if (fabsf(smoothed_error) > HEADING_DEADZONE && speed_pct > 20) {
                heading_correction_mm_s = pid_update(&pid_heading, 0.0f, smoothed_error);
            } else {
                // Reset integral to prevent windup
                pid_heading.integ = 0.0f;
                filter_init = false;  // Reset filter when in deadzone
            }
        }
    }

    // ========== 🔥 FIXED: Apply heading correction with CORRECT SIGN ==========
    float base_target = target_mm_s_from_ui();
    
    // CORRECTED LOGIC:
    // +error = target > current = drifted LEFT → need to turn RIGHT → SLOW LEFT, SPEED UP RIGHT
    // -error = target < current = drifted RIGHT → need to turn LEFT → SPEED UP LEFT, SLOW RIGHT
    //
    // So correction should be SUBTRACTED from left, ADDED to right
    float target_l = base_target - heading_correction_mm_s;
    float target_r = base_target + heading_correction_mm_s;
    
    // ========== Control: Feedforward + PID per wheel ==========
    pid_l.dt_s = dt_s;
    pid_r.dt_s = dt_s;
    
    // Left wheel: FF + PID
    float ff_l = feedforward_left(target_l);
    float pid_out_l = pid_update(&pid_l, target_l, vel_l);
    float pwm_l = ff_l + pid_out_l;
    
    // Right wheel: FF + PID
    float ff_r = feedforward_right(target_r);
    float pid_out_r = pid_update(&pid_r, target_r, vel_r);
    float pwm_r = ff_r + pid_out_r;
    
    // Clamp to limits
    if (pwm_l > 100.0f) pwm_l = 100.0f;
    if (pwm_l < -100.0f) pwm_l = -100.0f;
    if (pwm_r > 100.0f) pwm_r = 100.0f;
    if (pwm_r < -100.0f) pwm_r = -100.0f;
    
    // PWM slew-rate limiter
    static float prev_pwm_l = 0.0f, prev_pwm_r = 0.0f;
    const float MAX_PWM_STEP_PER_TICK = 5.0f;

    float d_l = pwm_l - prev_pwm_l;
    if (d_l >  MAX_PWM_STEP_PER_TICK) d_l =  MAX_PWM_STEP_PER_TICK;
    if (d_l < -MAX_PWM_STEP_PER_TICK) d_l = -MAX_PWM_STEP_PER_TICK;
    pwm_l = prev_pwm_l + d_l;

    float d_r = pwm_r - prev_pwm_r;
    if (d_r >  MAX_PWM_STEP_PER_TICK) d_r =  MAX_PWM_STEP_PER_TICK;
    if (d_r < -MAX_PWM_STEP_PER_TICK) d_r = -MAX_PWM_STEP_PER_TICK;
    pwm_r = prev_pwm_r + d_r;

    prev_pwm_l = pwm_l;
    prev_pwm_r = pwm_r;
    
    motor_set_signed(pwm_r, pwm_l);

    // ========== Status ==========
    if ((now - last_print_ms) >= DISPLAY_INTERVAL_MS) {
        float err_l = target_l - vel_l;
        float err_r = target_r - vel_r;
        
        printf("╔════════════════════════════════════════════════════════════════╗\n");
        printf("║ Base Target: %+.1f mm/s (%u%% %s)                             \n", 
               base_target, speed_pct, forward ? "FWD" : "REV");
        printf("╠════════════════════════════════════════════════════════════════╣\n");
        
        // Left wheel
        printf("║ LEFT:  Vel=%+5.1f | Target=%+5.1f | Err=%+5.1f | PWM=%+5.1f%%  ║\n", 
               vel_l, target_l, err_l, pwm_l);
        printf("║        FF=%+4.1f%% | PID=%+5.1f | I=%+5.1f | Dist=%.0fmm      ║\n",
               ff_l, pid_out_l, pid_l.integ, total_dist_l_mm);
        
        printf("╟────────────────────────────────────────────────────────────────╢\n");
        
        // Right wheel
        printf("║ RIGHT: Vel=%+5.1f | Target=%+5.1f | Err=%+5.1f | PWM=%+5.1f%%  ║\n", 
               vel_r, target_r, err_r, pwm_r);
        printf("║        FF=%+4.1f%% | PID=%+5.1f | I=%+5.1f | Dist=%.0fmm      ║\n",
               ff_r, pid_out_r, pid_r.integ, total_dist_r_mm);
        
        printf("╟────────────────────────────────────────────────────────────────╢\n");
        
        // IMU status
        if (imu_ready && imu_read_ok) {
            printf("║ IMU:   Heading=%5.1f° | Target=%5.1f° | Error=%+5.1f°      ║\n",
                   current_heading, target_heading, heading_error);
            printf("║        Correction=%+5.1f mm/s | I=%+5.1f                    ║\n",
                   heading_correction_mm_s, pid_heading.integ);
            
            // More detailed status
            if (fabsf(heading_error) < HEADING_DEADZONE) {
                printf("║        Status: ✅ ON TRACK (within %.1f° deadzone)           ║\n", HEADING_DEADZONE);
            } else {
                const char* dir_str = (heading_error > 0) ? "LEFT" : "RIGHT";
                printf("║        Status: 🔄 Drifted %s, correcting...                 ║\n", dir_str);
            }
        } else if (imu_ready) {
            printf("║ IMU:   ⚠️  Read failed or not locked yet                       ║\n");
        } else {
            printf("║ IMU:   ✗ DISABLED (fallback mode)                             ║\n");
        }
        
        printf("╟────────────────────────────────────────────────────────────────╢\n");
        
        // Tuning hints
        float avg_err = (fabsf(err_l) + fabsf(err_r)) / 2.0f;
        float dist_diff = fabsf(total_dist_l_mm - total_dist_r_mm);
        
        if (avg_err < 10.0f && dist_diff < 50.0f) {
            printf("║ ✅ Excellent! Both velocity & heading tracking are good!      ║\n");
        } else if (avg_err > 20.0f) {
            printf("║ ⚠️  Large velocity error - check wheel PID tuning             ║\n");
        } else if (dist_diff > 100.0f && imu_ready) {
            printf("║ 💡 IMU actively correcting path (wheel drift Δ=%.0fmm)        ║\n", dist_diff);
        } else {
            printf("║ 💡 System operating normally                                   ║\n");
        }
        
        printf("╚════════════════════════════════════════════════════════════════╝\n\n");
        
        last_print_ms = now;
    }
}