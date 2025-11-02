#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "config.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "control/pid.h"
#include "fsm.h"

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

static pid_t pid_r, pid_l;

#define VEL_WINDOW_MS 150
static uint32_t vel_window_start = 0;
static uint32_t vel_l_start = 0;
static uint32_t vel_r_start = 0;
static float vel_l = 0.0f;
static float vel_r = 0.0f;

static float total_dist_l_mm = 0.0f;
static float total_dist_r_mm = 0.0f;

void fsm_init(void) {
    stdio_init_all();
    motor_init_all();
    encoder_init();

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
    
    // Initialize LEFT wheel PID with its own gains
    pid_init(&pid_l, PID_L_KP, PID_L_KI, PID_L_KD, dt,
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);
    
    // Initialize RIGHT wheel PID with its own gains
    pid_init(&pid_r, PID_R_KP, PID_R_KI, PID_R_KD, dt,
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);

    motor_set_signed(0, 0);

    vel_l_start = encoder_left_count();
    vel_r_start = encoder_right_count();
    vel_window_start = now_ms();
    last_control_ms = vel_window_start;

    printf("\n╔════════════════════════════════════════════════════════╗\n");
    printf("║  PER-WHEEL PID WITH FEEDFORWARD + DEADBAND COMPENSATION║\n");
    printf("╚════════════════════════════════════════════════════════╝\n\n");
    printf("Encoder: CPR=%.0f | Wheel=%.1fmm | Resolution=%.2fmm\n",
           ENCODER_CPR, WHEEL_DIAMETER_MM, encoder_mm_per_tick());
    printf("Velocity window: %dms\n", VEL_WINDOW_MS);
    printf("Deadband compensation: %.0f%%\n\n", MOTOR_DEADBAND_PERCENT);
    
    printf("┌─ LEFT MOTOR ────────────────────────────────────┐\n");
    printf("│ Feedforward: %.1f%% to %.1f%%                    │\n", 
           MOTOR_L_MIN_PWM, MOTOR_L_MAX_PWM);
    printf("│ PID: Kp=%.2f Ki=%.3f Kd=%.3f                   │\n", 
           PID_L_KP, PID_L_KI, PID_L_KD);
    printf("└─────────────────────────────────────────────────┘\n\n");
    
    printf("┌─ RIGHT MOTOR ───────────────────────────────────┐\n");
    printf("│ Feedforward: %.1f%% to %.1f%%                    │\n", 
           MOTOR_R_MIN_PWM, MOTOR_R_MAX_PWM);
    printf("│ PID: Kp=%.2f Ki=%.3f Kd=%.3f                   │\n", 
           PID_R_KP, PID_R_KI, PID_R_KD);
    printf("└─────────────────────────────────────────────────┘\n\n");
    
    printf("Target Speed: %u%% = %.1f mm/s\n", 
           speed_pct, SPEED_MAX_MM_S * speed_pct / 100.0f);
    printf("GP21=dir | GP20=speed\n\n");
}

static float target_mm_s_from_ui(void) {
    float sign = forward ? +1.0f : -1.0f;
    return sign * (SPEED_MAX_MM_S * (speed_pct / 100.0f));
}

// Feedforward for LEFT wheel (with deadband compensation)

// Helper: map target speed to PWM% with fading deadband
static inline float ff_from_speed(float target_mm_s, float min_pwm, float max_pwm) {
    float s = target_mm_s;
    float sign = (s >= 0.0f) ? 1.0f : -1.0f;
    float mag = fabsf(s);

    float base = (SPEED_MAX_MM_S > 1.0f) ? (mag / SPEED_MAX_MM_S) : 0.0f;
    if (base < 0.0f) base = 0.0f;
    if (base > 1.0f) base = 1.0f;

    float pwm = min_pwm + (max_pwm - min_pwm) * base;

    float fade = 1.0f - base;                     // 1 at zero speed → 0 at max
    float add  = (MOTOR_DEADBAND_PERCENT * 0.4f) * fade; // use 40% of configured deadband
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


// Feedforward for RIGHT wheel (with deadband compensation)
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
    
    // ========== Buttons ==========
    if (g_toggle_dir_req) {
        g_toggle_dir_req = false;
        forward = !forward;
        pid_reset(&pid_r); 
        pid_reset(&pid_l);
        vel_l_start = encoder_left_count();
        vel_r_start = encoder_right_count();
        vel_window_start = now;
        vel_l = vel_r = 0.0f;
        printf("\n>>> DIRECTION: %s <<<\n\n", forward ? "FWD" : "REV");
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

        // Exponential Moving Average smoothing
        static bool vel_ema_init = false;
        static float vel_l_ema = 0.0f, vel_r_ema = 0.0f;
        const float alpha_vel = 0.40f; // lower = smoother

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

    // ========== Control: Feedforward (with deadband) + PID ==========
    pid_l.dt_s = dt_s;
    pid_r.dt_s = dt_s;
    
    float target = target_mm_s_from_ui();
    
    // Left wheel: Feedforward (includes deadband) + PID correction
    float ff_l = feedforward_left(target);
    float pid_out_l = pid_update(&pid_l, target, vel_l);
    float pwm_l = ff_l + pid_out_l;
    
    // Right wheel: Feedforward (includes deadband) + PID correction
    float ff_r = feedforward_right(target);
    float pid_out_r = pid_update(&pid_r, target, vel_r);
    float pwm_r = ff_r + pid_out_r;
    
    // Clamp to limits
    if (pwm_l > 100.0f) pwm_l = 100.0f;
    if (pwm_l < -100.0f) pwm_l = -100.0f;
    if (pwm_r > 100.0f) pwm_r = 100.0f;
    if (pwm_r < -100.0f) pwm_r = -100.0f;

    
    // --- PWM slew-rate limiter ---
    static float prev_pwm_l = 0.0f, prev_pwm_r = 0.0f;
    const float MAX_PWM_STEP_PER_TICK = 5.0f; // ±2%% per control tick

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
        float err_l = target - vel_l;
        float err_r = target - vel_r;
        
        printf("╔══════════════════════════════════════════════════════════╗\n");
        printf("║ Target: %+.1f mm/s (%u%% %s)                            \n", 
               target, speed_pct, forward ? "FWD" : "REV");
        printf("╠══════════════════════════════════════════════════════════╣\n");
        
        // Left wheel
        printf("║ LEFT:  Vel=%+5.1f | Err=%+5.1f | FF=%+4.1f%% | PID=%+5.1f   ║\n", 
               vel_l, err_l, ff_l, pid_out_l);
        printf("║        PWM=%+5.1f%% | I=%+5.1f | Dist=%.0fmm             ║\n",
               pwm_l, pid_l.integ, total_dist_l_mm);
        
        printf("╟──────────────────────────────────────────────────────────╢\n");
        
        // Right wheel
        printf("║ RIGHT: Vel=%+5.1f | Err=%+5.1f | FF=%+4.1f%% | PID=%+5.1f   ║\n", 
               vel_r, err_r, ff_r, pid_out_r);
        printf("║        PWM=%+5.1f%% | I=%+5.1f | Dist=%.0fmm             ║\n",
               pwm_r, pid_r.integ, total_dist_r_mm);
        
        printf("╟──────────────────────────────────────────────────────────╢\n");
        
        // Tuning hints
        float avg_err = (fabsf(err_l) + fabsf(err_r)) / 2.0f;
        
        if (avg_err < 5.0f) {
            printf("║ ✅ Excellent! Avg error <5mm/s - Well tuned!             ║\n");
        } else if (avg_err > 15.0f) {
            printf("║ ⚠️  Large error! Check feedforward first:                ║\n");
            if (fabsf(err_l) > 15.0f) {
                if (err_l > 0) {
                    printf("║    Left too slow → Increase MOTOR_L_MAX_PWM (%.1f)       ║\n", MOTOR_L_MAX_PWM);
                } else {
                    printf("║    Left too fast → Decrease MOTOR_L_MAX_PWM (%.1f)       ║\n", MOTOR_L_MAX_PWM);
                }
            }
            if (fabsf(err_r) > 15.0f) {
                if (err_r > 0) {
                    printf("║    Right too slow → Increase MOTOR_R_MAX_PWM (%.1f)      ║\n", MOTOR_R_MAX_PWM);
                } else {
                    printf("║    Right too fast → Decrease MOTOR_R_MAX_PWM (%.1f)      ║\n", MOTOR_R_MAX_PWM);
                }
            }
        } else {
            printf("║ 💡 Feedforward OK. Now tune PID:                         ║\n");
            if (fabsf(err_l) > 5.0f) {
                printf("║    Left error=%.1f → Adjust PID_L_KP (now %.2f)          ║\n", err_l, PID_L_KP);
            }
            if (fabsf(err_r) > 5.0f) {
                printf("║    Right error=%.1f → Adjust PID_R_KP (now %.2f)         ║\n", err_r, PID_R_KP);
            }
        }
        
        printf("╚══════════════════════════════════════════════════════════╝\n\n");
        
        last_print_ms = now;
    }
}