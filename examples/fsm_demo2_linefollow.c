#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "config/config.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/ir_line_follower.h"
#include "drivers/magnetometer.h"
#include "control/pid.h"
#include "fsm.h"

// ======================================================================
// Line Following + IMU Smoothing - Maintains heading while following line
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
static uint speed_pct = 25;
static uint32_t last_print_ms = 0;
static uint32_t last_control_ms = 0;
static uint32_t last_sensor_print_ms = 0;

static pid_t pid_r, pid_l, pid_line, pid_heading;

#define VEL_WINDOW_MS 150
static uint32_t vel_window_start = 0;
static uint32_t vel_l_start = 0;
static uint32_t vel_r_start = 0;
static float vel_l = 0.0f;
static float vel_r = 0.0f;

static float total_dist_l_mm = 0.0f;
static float total_dist_r_mm = 0.0f;

// Line following target
#define IR_LINE_EDGE_TARGET ((IR_LINE_EDGE_LOW_LIMIT + IR_LINE_EDGE_HIGH_LIMIT) / 2.0f)
#define LINE_ERROR_CLAMP_MAX 400.0f
#define LINE_FOLLOW_MAX_SPEED 30

// Smoothing filter for IR sensor
static float ir_filtered = IR_LINE_EDGE_TARGET;

// Memory for detecting stuck on white/black
static uint32_t white_time_start = 0;
static bool was_on_white = false;

// IMU/Heading tracking
static float target_heading = 0.0f;
static bool heading_locked = false;
static bool imu_ready = false;
static uint32_t last_heading_update_ms = 0;

// Startup state machine for IMU
typedef enum {
    STATE_IMU_WARMUP,
    STATE_HEADING_LOCK,
    STATE_RUNNING
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
    sleep_ms(2000);
    
    printf("\n========== SYSTEM STARTING ==========\n");
    
    motor_init_all();
    encoder_init();
    ir_line_follower_init();

    // Initialize magnetometer
    if (!magnetometer_init()) {
        printf("⚠️  WARNING: Magnetometer init failed! Running without IMU smoothing.\n");
        imu_ready = false;
        startup_state = STATE_RUNNING;
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
    
    pid_init(&pid_l, PID_L_KP, PID_L_KI, PID_L_KD, dt,
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);
    
    pid_init(&pid_r, PID_R_KP, PID_R_KI, PID_R_KD, dt,
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);
    
    pid_init(&pid_line, LINE_FOLLOW_KP, LINE_FOLLOW_KI, LINE_FOLLOW_KD, dt,
             -LINE_CORRECTION_MAX, LINE_CORRECTION_MAX, -3.0f, 3.0f);

    // IMU heading correction PID (gentle to avoid fighting line following)
    pid_init(&pid_heading, HEADING_KP * 0.3f, HEADING_KI * 0.3f, HEADING_KD * 0.3f, dt,
             -MAX_HEADING_CORRECTION * 0.5f, MAX_HEADING_CORRECTION * 0.5f, -10.0f, 10.0f);

    motor_set_signed(0, 0);

    vel_l_start = encoder_left_count();
    vel_r_start = encoder_right_count();
    vel_window_start = now_ms();
    last_control_ms = vel_window_start;
    last_sensor_print_ms = vel_window_start;

    printf("\n╔════════════════════════════════════════════════════════╗\n");
    printf("║   LINE FOLLOWING WITH IMU SMOOTHING (MEMORY MODE)     ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n\n");
    
    printf("┌─ LINE FOLLOWING (MEMORY MODE) ──────────────────────┐\n");
    printf("│ IR Sensor: GP%d (ADC ch%d)                          │\n", 
           IR_LINE_ADC_GPIO, IR_LINE_ADC_CHANNEL);
    printf("│ ✓ CALIBRATED Edge: %d - %d (Tgt: %.0f)            │\n", 
           IR_LINE_EDGE_LOW_LIMIT, IR_LINE_EDGE_HIGH_LIMIT, IR_LINE_EDGE_TARGET);
    printf("│ Sensor Filter: %.1f%% (smoothing)                   │\n", 
           IR_SENSOR_FILTER_ALPHA * 100.0f);
    printf("│ Base PID: Kp=%.4f Ki=%.4f Kd=%.3f                  │\n", 
           LINE_FOLLOW_KP, LINE_FOLLOW_KI, LINE_FOLLOW_KD);
    printf("│ Adaptive: 0.1× gentle → 1.25× aggressive           │\n");
    printf("│ Memory Boost: Activates on prolonged white/black   │\n");
    printf("│ Max Correction: %.0f%% | Max Speed: %d%%             │\n", 
           LINE_CORRECTION_MAX, LINE_FOLLOW_MAX_SPEED);
    printf("└──────────────────────────────────────────────────────┘\n\n");
    
    printf("┌─ IMU HEADING SMOOTHING ──────────────────────────────┐\n");
    printf("│ Purpose: Compensate for wheel drift & wobble        │\n");
    printf("│ Kp=%.2f Ki=%.2f Kd=%.2f (gentle)                │\n", 
           HEADING_KP * 0.3f, HEADING_KI * 0.3f, HEADING_KD * 0.3f);
    printf("│ Max Correction: ±%.0fmm/s | Deadzone: %.1f°        │\n", 
           MAX_HEADING_CORRECTION * 0.5f, HEADING_DEADZONE);
    printf("│ Status: %s                                          │\n",
           imu_ready ? "READY" : "DISABLED");
    printf("└──────────────────────────────────────────────────────┘\n\n");
    
    if (imu_ready) {
        printf("⏳ Warming up IMU filter (%d samples)...\n\n", IMU_WARMUP_SAMPLES);
    }
    
    printf("💡 How it works:\n");
    printf("   - Line sensor: PRIMARY control (dominates)\n");
    printf("   - IMU: SECONDARY smoothing (gentle corrections)\n");
    printf("   - Result: Smoother, steadier line following\n\n");
    
    printf("Current Speed: %d%% | Max: %d%%\n", speed_pct, LINE_FOLLOW_MAX_SPEED);
    printf("GP21=dir (lock heading) | GP20=speed\n");
    printf("========== READY ==========\n\n");
}

static float target_mm_s_from_ui(void) {
    uint effective_speed = speed_pct;
    if (effective_speed > LINE_FOLLOW_MAX_SPEED) {
        effective_speed = LINE_FOLLOW_MAX_SPEED;
    }
    
    float sign = forward ? +1.0f : -1.0f;
    return sign * (SPEED_MAX_MM_S * (effective_speed / 100.0f));
}

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
    
    // ========== STARTUP STATE MACHINE (IMU WARMUP) ==========
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
        pid_reset(&pid_line);
        pid_reset(&pid_heading);
        vel_l_start = encoder_left_count();
        vel_r_start = encoder_right_count();
        vel_window_start = now;
        vel_l = vel_r = 0.0f;
        ir_filtered = IR_LINE_EDGE_TARGET;
        white_time_start = 0;
        was_on_white = false;
        
        // Lock new heading on direction change
        if (imu_ready && startup_state == STATE_RUNNING) {
            magnetometer_data_t mag_data;
            if (magnetometer_read_data(&mag_data)) {
                target_heading = mag_data.heading;
                heading_locked = true;
                printf("\n>>> DIR: %s | Heading: %.1f° <<<\n\n", 
                       forward ? "FWD" : "REV", target_heading);
            }
        } else {
            printf("\n>>> DIRECTION: %s <<<\n\n", forward ? "FWD" : "REV");
        }
    }
    if (g_change_speed_req) {
        g_change_speed_req = false;
        speed_pct = 25 + (rand() % 16);
        printf("\n>>> SPEED: %u%% (max %d%%) <<<\n\n", speed_pct, LINE_FOLLOW_MAX_SPEED);
    }

    // ========== Read IR Sensor with SMOOTHING FILTER ==========
    uint16_t ir_raw = ir_line_read_adc_averaged(8);
    
    // Apply exponential moving average filter
    ir_filtered = IR_SENSOR_FILTER_ALPHA * ir_raw + (1.0f - IR_SENSOR_FILTER_ALPHA) * ir_filtered;
    
    // Use filtered value for control
    float line_error_raw = ir_filtered - IR_LINE_EDGE_TARGET;
    
    // Clamp error
    float line_error = line_error_raw;
    if (line_error > LINE_ERROR_CLAMP_MAX) {
        line_error = LINE_ERROR_CLAMP_MAX;
    }
    if (line_error < -LINE_ERROR_CLAMP_MAX) {
        line_error = -LINE_ERROR_CLAMP_MAX;
    }
    
    // ========== ADAPTIVE PID GAIN WITH MEMORY ==========
    float error_magnitude = fabsf(line_error);
    float adaptive_kp;
    
    // Detect if robot is stuck seeing white OR black for too long
    bool currently_on_white = (ir_filtered < IR_LINE_EDGE_LOW_LIMIT);
    bool currently_on_black = (ir_filtered > IR_LINE_EDGE_HIGH_LIMIT);
    
    // Track time on white/black
    static uint32_t black_time_start = 0;
    static bool was_on_black = false;
    uint32_t white_duration = 0;
    uint32_t black_duration = 0;
    
    // WHITE memory tracking
    if (currently_on_white) {
        if (!was_on_white) {
            white_time_start = now;
            was_on_white = true;
        } else {
            white_duration = now - white_time_start;
        }
        was_on_black = false;
        black_time_start = 0;
    } 
    // BLACK memory tracking
    else if (currently_on_black) {
        if (!was_on_black) {
            black_time_start = now;
            was_on_black = true;
        } else {
            black_duration = now - black_time_start;
        }
        was_on_white = false;
        white_time_start = 0;
    }
    // ON EDGE - reset both
    else {
        was_on_white = false;
        was_on_black = false;
        white_time_start = 0;
        black_time_start = 0;
    }
    
    // Base adaptive gain
    if (error_magnitude < 50.0f) {
        adaptive_kp = LINE_FOLLOW_KP * 0.1f;
    } else if (error_magnitude < 100.0f) {
        adaptive_kp = LINE_FOLLOW_KP * 0.2f;
    } else if (error_magnitude < 200.0f) {
        adaptive_kp = LINE_FOLLOW_KP * 0.5f;
    } else if (error_magnitude < 400.0f) {
        adaptive_kp = LINE_FOLLOW_KP * 1.0f;
    } else {
        adaptive_kp = LINE_FOLLOW_KP * 1.25f;
    }
    
    // MEMORY BOOST
    float time_boost = 1.0f;
    uint32_t stuck_duration = 0;
    const char* stuck_on = "";
    
    if (white_duration > 500) {  // REDUCED from 400ms - react faster
        stuck_duration = white_duration;
        stuck_on = "WHITE";
        // More aggressive boost formula: starts at 3× and ramps up quickly
        time_boost = 12.0f + ((white_duration - 100) / 200.0f);
        if (time_boost > 30.0f) time_boost = 30.0f;  // Higher cap for emergency
        
        adaptive_kp *= time_boost;
        pid_line.out_max = LINE_CORRECTION_MAX * time_boost;
        pid_line.out_min = -LINE_CORRECTION_MAX * time_boost;
    } 
    else if (black_duration > 50) {
        stuck_duration = black_duration;
        stuck_on = "BLACK";
        // Also more aggressive for black
        time_boost = 6.0f + ((black_duration - 50) / 150.0f);
        if (time_boost > 10.0f) time_boost = 10.0f;
        
        adaptive_kp *= time_boost;
        pid_line.out_max = LINE_CORRECTION_MAX * time_boost;
        pid_line.out_min = -LINE_CORRECTION_MAX * time_boost;
    } 
    else {
        pid_line.out_max = LINE_CORRECTION_MAX;
        pid_line.out_min = -LINE_CORRECTION_MAX;
    }
    
    // Apply adaptive gain to line PID
    float original_kp = pid_line.kp;
    pid_line.kp = adaptive_kp;
    float line_correction = pid_update(&pid_line, 0.0f, line_error);
    pid_line.kp = original_kp;
    
    bool on_edge = (ir_filtered >= IR_LINE_EDGE_LOW_LIMIT && ir_filtered <= IR_LINE_EDGE_HIGH_LIMIT);

    // ========== Read IMU for GENTLE Heading Smoothing ==========
    float heading_correction_mm_s = 0.0f;
    float current_heading = 0.0f;
    float heading_error = 0.0f;
    bool imu_read_ok = false;
    
    // Only update heading when on edge (stabilize, don't fight line corrections)
    if (imu_ready && heading_locked && startup_state == STATE_RUNNING && on_edge) {
        magnetometer_data_t mag_data;
        if (magnetometer_read_data(&mag_data)) {
            current_heading = mag_data.heading;
            heading_error = normalize_angle_error(target_heading - current_heading);
            imu_read_ok = true;
            
            // Low-pass filter on heading error (heavy smoothing)
            static float filtered_heading_error = 0.0f;
            static bool filter_init = false;
            const float alpha_heading = 0.2f;  // Very smooth
            
            if (!filter_init) {
                filtered_heading_error = heading_error;
                filter_init = true;
            } else {
                filtered_heading_error = alpha_heading * heading_error + 
                                        (1.0f - alpha_heading) * filtered_heading_error;
            }
            
            // Only apply gentle correction when stable and outside deadzone
            if (fabsf(filtered_heading_error) > HEADING_DEADZONE * 2.0f && speed_pct > 20) {
                heading_correction_mm_s = pid_update(&pid_heading, 0.0f, filtered_heading_error);
            } else {
                pid_heading.integ = 0.0f;
                if (fabsf(filtered_heading_error) < HEADING_DEADZONE) {
                    filter_init = false;
                }
            }
            
            last_heading_update_ms = now;
        }
    }

    // ========== Debug Output ==========
    if ((now - last_sensor_print_ms) >= 300) {
        const char* status = on_edge ? "✓EDGE" : (ir_filtered < IR_LINE_EDGE_LOW_LIMIT ? " WHITE" : " BLACK");
        const char* boost_indicator = "";
        
        if (white_duration > 300 || black_duration > 50) {
            boost_indicator = " 🚨BOOST";
        }
        
        printf("[IR] Raw:%4u→Filt:%4.0f | Err:%+4.0f | Kp×%.1f | LCorr:%+5.1f%% | %s%s",
               ir_raw, ir_filtered, line_error, 
               (adaptive_kp / LINE_FOLLOW_KP), line_correction, status, boost_indicator);
        
        if (imu_read_ok && fabsf(heading_correction_mm_s) > 0.1f) {
            printf(" | [IMU] E:%+.1f° HCorr:%+.1f", heading_error, heading_correction_mm_s);
        }
        
        if (stuck_duration > 50) {
            printf(" (%.1fs on %s)", stuck_duration / 1000.0f, stuck_on);
        }
        printf("\n");
        
        last_sensor_print_ms = now;
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

    // ========== Motor Control: Line + Gentle IMU Smoothing ==========
    pid_l.dt_s = dt_s;
    pid_r.dt_s = dt_s;
    pid_line.dt_s = dt_s;
    
    float base_target = target_mm_s_from_ui();
    
    // Combine corrections (line dominates, IMU provides gentle smoothing)
    // Line correction: WHITE → turn LEFT (+ left, - right)
    // Heading correction: drift left → turn RIGHT (- left, + right)
    float target_l = base_target + line_correction - heading_correction_mm_s;
    float target_r = base_target - line_correction + heading_correction_mm_s;
    
    // Calculate PWM
    float ff_l = feedforward_left(target_l);
    float pid_out_l = pid_update(&pid_l, target_l, vel_l);
    float pwm_l = ff_l + pid_out_l;
    
    float ff_r = feedforward_right(target_r);
    float pid_out_r = pid_update(&pid_r, target_r, vel_r);
    float pwm_r = ff_r + pid_out_r;
    
    // Clamp
    if (pwm_l > 100.0f) pwm_l = 100.0f;
    if (pwm_l < -100.0f) pwm_l = -100.0f;
    if (pwm_r > 100.0f) pwm_r = 100.0f;
    if (pwm_r < -100.0f) pwm_r = -100.0f;
    
    // Slew rate limiter
    static float prev_pwm_l = 0.0f, prev_pwm_r = 0.0f;
    const float MAX_PWM_STEP_PER_TICK = 1.5f;

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
        
        uint effective_speed = speed_pct > LINE_FOLLOW_MAX_SPEED ? LINE_FOLLOW_MAX_SPEED : speed_pct;
        
        printf("\n╔══════════════════════════════════════════════════════════╗\n");
        printf("║ LINE + IMU MODE | Speed: %u%%→%u%% | %.1f mm/s %s       ║\n", 
               speed_pct, effective_speed, base_target, forward ? "FWD" : "REV");
        printf("╠══════════════════════════════════════════════════════════╣\n");
        printf("║ LINE: Raw=%4u | Filt=%.0f | Err=%+.0f | Corr=%+.1f%%    ║\n",
               ir_raw, ir_filtered, line_error, line_correction);
        
        if (imu_read_ok) {
            printf("║ IMU:  H=%.1f°→%.1f° | Err=%+.1f° | Corr=%+.1fmm/s       ║\n",
                   current_heading, target_heading, heading_error, heading_correction_mm_s);
            
            if (fabsf(heading_correction_mm_s) < 0.5f) {
                printf("║       Status: ✅ Stable (minimal IMU correction)          ║\n");
            } else {
                printf("║       Status: 🔄 Smoothing (compensating drift)           ║\n");
            }
        } else {
            printf("║ IMU:  %s                                                  ║\n",
                   imu_ready ? (on_edge ? "Monitoring..." : "Standby (off edge)") : "DISABLED");
        }
        
        if (white_duration > 300 || black_duration > 50) {
            printf("║ 🚨 MEMORY BOOST: %.1fs on %s | Boost: %.1f×             ║\n",
                   stuck_duration / 1000.0f, stuck_on, time_boost);
        }
        
        printf("╟──────────────────────────────────────────────────────────╢\n");
        printf("║ L: Vel=%+5.1f | Tgt=%+5.1f | PWM=%+5.1f%% | Err=%+5.1f  ║\n", 
               vel_l, target_l, pwm_l, err_l);
        printf("║ R: Vel=%+5.1f | Tgt=%+5.1f | PWM=%+5.1f%% | Err=%+5.1f  ║\n", 
               vel_r, target_r, pwm_r, err_r);
        printf("╟──────────────────────────────────────────────────────────╢\n");
        
        if (on_edge && fabsf(line_error) < 50.0f) {
            printf("║ ✅ PERFECT - Smooth tracking with IMU assist!            ║\n");
        } else if (on_edge) {
            printf("║ 💡 On edge - following OK                                 ║\n");
        } else {
            if (ir_filtered < IR_LINE_EDGE_LOW_LIMIT) {
                printf("║ ⚠️  Off line (WHITE) - Turning LEFT | Kp×%.1f            ║\n",
                       (adaptive_kp / LINE_FOLLOW_KP));
            } else {
                printf("║ ⚠️  Off line (BLACK) - Turning RIGHT | Kp×%.1f           ║\n",
                       (adaptive_kp / LINE_FOLLOW_KP));
            }
        }
        
        printf("╚══════════════════════════════════════════════════════════╝\n\n");
        
        last_print_ms = now;
    }
}