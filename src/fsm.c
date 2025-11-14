#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"

#include "config.h"
#include "fsm.h"

#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/ir_line_follower.h"
#include "drivers/ir_barcode_scanner.h"
#include "drivers/magnetometer.h"
#include "drivers/ultrasonic.h"
#include "control/pid.h"

// --------- helpers / state ----------
static inline uint32_t now_ms(void){ return to_ms_since_boot(get_absolute_time()); }
static inline float clampf(float x, float lo, float hi){ return (x < lo) ? lo : (x > hi ? hi : x); }

// Buttons
static volatile bool g_toggle_dir_req = false;
static volatile bool g_change_speed_req = false;
static volatile uint32_t g_last_dir_ms = 0;
static volatile uint32_t g_last_spd_ms = 0;

// UI state
static bool  forward   = true;
static uint  speed_pct = 50;

// timekeeping
static uint32_t last_control_ms = 0;
static uint32_t last_print_ms   = 0;

// wheel PIDs + heading + line
static pid_t pid_r, pid_l, pid_heading;

#define VEL_WINDOW_MS 150
static uint32_t vel_window_start = 0;
static uint32_t vel_l_start = 0, vel_r_start = 0;
static float    vel_l = 0.0f,  vel_r = 0.0f;
static float    total_dist_l_mm = 0.0f, total_dist_r_mm = 0.0f;

// IMU state
static float target_heading = 0.0f;
static bool  heading_locked = false;
static bool  imu_ready      = false;

// barcode state
static bool barcode_enabled     = false;
static char last_barcode_char   = '\0';
char ch;
static int  last_bars_count     = 0;

// startup mini-FSM
typedef enum { STATE_IMU_WARMUP, STATE_HEADING_LOCK, STATE_RUNNING } startup_state_t;
static startup_state_t startup_state = STATE_IMU_WARMUP;
static uint8_t imu_samples = 0;
#define IMU_WARMUP_SAMPLES 10

// last telemetry snapshot
static fsm_telemetry_t g_tm = {0};

// angle helper
static float normalize_angle_error(float e){
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

// target speed (with direction)
static inline float target_mm_s_from_ui(void) {
    float sign = forward ? +1.0f : -1.0f;
    return sign * (SPEED_MAX_MM_S * (speed_pct / 100.0f));
}

// feedforward helpers
static inline float ff_from_speed(float target_mm_s, float min_pwm, float max_pwm) {
    float sign = (target_mm_s >= 0.0f) ? 1.0f : -1.0f;
    float base = fabsf(target_mm_s) / SPEED_MAX_MM_S; // 0..1
    if (base < 0.0f) base = 0.0f; if (base > 1.0f) base = 1.0f;
    float pwm  = min_pwm + (max_pwm - min_pwm) * base;
    float fade = 1.0f - base;
    pwm += (MOTOR_DEADBAND_PERCENT * 0.4f) * fade;
    return sign * pwm;
}
static inline float feedforward_left (float t){ float p = ff_from_speed(t, MOTOR_L_MIN_PWM, MOTOR_L_MAX_PWM); float lim = (p>=0)?MOTOR_L_MAX_PWM:-MOTOR_L_MAX_PWM; return clampf(p,-fabsf(lim),fabsf(lim)); }
static inline float feedforward_right(float t){ float p = ff_from_speed(t, MOTOR_R_MIN_PWM, MOTOR_R_MAX_PWM); float lim = (p>=0)?MOTOR_R_MAX_PWM:-MOTOR_R_MAX_PWM; return clampf(p,-fabsf(lim),fabsf(lim)); }

// encoder + buttons ISR
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

// Map A..Z to ±90° turn (Right for A,C,E.. ; Left for B,D,F..)
static int turn_dir_from_char(char ch){
    switch (ch){
        case 'A': case 'C': case 'E': case 'G': case 'I': case 'K': case 'M': case 'O': case 'Q': case 'S': case 'U': case 'W': case 'Y':
            return +1; // right
        case 'B': case 'D': case 'F': case 'H': case 'J': case 'L': case 'N': case 'P': case 'R': case 'T': case 'V': case 'X': case 'Z':
            return -1; // left
        default: return 0;
    }
}

// --------- public API ----------
void fsm_init(void) {
    motor_init_all();
    encoder_init();
    ir_line_follower_init();
    ultrasonic_init();

    // Barcode
    ir_barcode_scanner_init();
    barcode_enabled = true;
    ir_barcode_start_scan();

    // Magnetometer
    if (!magnetometer_init()) {
        printf("⚠️  Magnetometer init failed – IMU correction disabled.\n");
        imu_ready = false;
        startup_state = STATE_RUNNING;
    } else {
        imu_ready = true;
    }

    // Buttons
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

    // PID init
    float dt = 0.01f;
    pid_init(&pid_l, PID_L_KP, PID_L_KI, PID_L_KD, dt, PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);
    pid_init(&pid_r, PID_R_KP, PID_R_KI, PID_R_KD, dt, PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);
    pid_init(&pid_heading, HEADING_KP, HEADING_KI, HEADING_KD, dt, -MAX_HEADING_CORRECTION, +MAX_HEADING_CORRECTION, -20.0f, 20.0f);

    motor_set_signed(0, 0);

    vel_l_start = encoder_left_count();
    vel_r_start = encoder_right_count();
    vel_window_start = now_ms();
    last_control_ms = vel_window_start;

    // init telemetry
    g_tm.barcode_char = '\0';
    g_tm.bars_count   = 0;
}

void fsm_on_cmd(const char *topic, const uint8_t *payload, size_t len) {
    if (!topic) return;
    if (strstr(topic, "/speed")) {
        int v = atoi((const char*)payload);
        if      (v <  0) v = 0;
        else if (v > 100) v = 100;
        speed_pct = (uint)v;
        printf("[CMD] speed_pct=%u\n", speed_pct);
    } else if (strstr(topic, "/dir")) {
        if (len && (payload[0]=='r' || payload[0]=='R')) forward = false;
        else forward = true;
        printf("[CMD] forward=%d\n", (int)forward);
    }
}

void fsm_step(uint32_t now) {
    // rate limit
    uint32_t dt_ms = now - last_control_ms;
    if (dt_ms < 5) return;
    last_control_ms = now;
    float dt_s = dt_ms / 1000.0f;

    // ===== IMU warmup/lock =====
    if (imu_ready && startup_state != STATE_RUNNING) {
        magnetometer_data_t md;
        switch (startup_state) {
            case STATE_IMU_WARMUP:
                if (magnetometer_read_data(&md)) {
                    if (++imu_samples >= IMU_WARMUP_SAMPLES) {
                        startup_state = STATE_HEADING_LOCK;
                    }
                }
                motor_set_signed(0,0);
                sleep_ms(50);
                return;
            case STATE_HEADING_LOCK:
                if (magnetometer_read_data(&md)) {
                    target_heading = md.heading;
                    heading_locked = true;
                    startup_state  = STATE_RUNNING;
                }
                break;
            default: break;
        }
    }

    // ===== buttons =====
    if (g_toggle_dir_req) {
        g_toggle_dir_req = false;
        forward = !forward;
        pid_reset(&pid_l); pid_reset(&pid_r); pid_reset(&pid_heading);
        vel_l_start = encoder_left_count();
        vel_r_start = encoder_right_count();
        vel_window_start = now;
        vel_l = vel_r = 0.0f;

        if (imu_ready) {
            magnetometer_data_t md;
            if (magnetometer_read_data(&md)) { target_heading = md.heading; heading_locked = true; }
        }
    }
    if (g_change_speed_req) {
        g_change_speed_req = false;
        speed_pct = 40u + (rand() % 61); // 40..100
    }

    // ===== barcode scanner =====
    if (barcode_enabled) {
        ir_barcode_update();

        if (ir_barcode_has_char()) {
            ch = ir_barcode_get_char();
            if (ch != '\0' && ch != last_barcode_char) {
                last_barcode_char = ch;
                

                // optional: bar count if your driver provides it
                int bars = 0;
                #ifdef IR_BARCODE_HAS_BAR_COUNT
                bars = ir_barcode_get_bar_count();
                #endif
                last_bars_count = bars;
                printf("[BARCODE] detected '%c' (bars=%d)\n", ch, bars);

                // Execute ±90° turn using IMU (if available)
                int dir = turn_dir_from_char(ch);
                if (dir != 0 && imu_ready && heading_locked) {
                    magnetometer_data_t md;
                    if (magnetometer_read_data(&md)) {
                        float cur = md.heading;
                        float tgt = cur + (dir > 0 ? +90.0f : -90.0f);
                        while (tgt < 0.0f)   tgt += 360.0f;
                        while (tgt >= 360.0f) tgt -= 360.0f;

                        uint32_t t0 = now_ms();
                        bool done = false;
                        motor_set_signed(0,0);
                        sleep_ms(200);

                        while (!done && (now_ms() - t0) < TURN_TIMEOUT_MS) {
                            if (magnetometer_read_data(&md)) {
                                float err = tgt - md.heading;
                                while (err >  180.0f) err -= 360.0f;
                                while (err < -180.0f) err += 360.0f;

                                float aerr = fabsf(err);
                                if (aerr <= HEADING_TOLERANCE) {
                                    motor_set_signed(0,0);
                                    done = true;
                                    break;
                                }
                                float turn_pwm = (aerr < APPROACH_THRESHOLD) ? MIN_TURN_SPEED : MAX_TURN_SPEED;
                                if (err > 0) motor_set_signed(-turn_pwm, +turn_pwm); // right
                                else         motor_set_signed(+turn_pwm, -turn_pwm); // left
                            }
                            sleep_ms(10);
                        }
                        motor_set_signed(0,0);
                        target_heading = tgt; // new forward heading
                        sleep_ms(200);
                    }
                }

                // ready for next char
                ir_barcode_clear_char();
                ir_barcode_reset();
                ir_barcode_start_scan();
            }
        }
    }

    // ===== velocity window =====
    uint32_t win_elapsed = now - vel_window_start;
    if (win_elapsed >= VEL_WINDOW_MS) {
        uint32_t cur_l = encoder_left_count();
        uint32_t cur_r = encoder_right_count();
        int32_t ticks_l = (int32_t)(cur_l - vel_l_start);
        int32_t ticks_r = (int32_t)(cur_r - vel_r_start);

        float mm_per_tick = encoder_mm_per_tick();
        float win_s = win_elapsed / 1000.0f;

        float inst_l = (ticks_l * mm_per_tick) / win_s;
        float inst_r = (ticks_r * mm_per_tick) / win_s;

        static bool ema_init=false; static float ema_l=0.0f, ema_r=0.0f;
        const float alpha = 0.40f;
        if (!ema_init){ ema_l=inst_l; ema_r=inst_r; ema_init=true; }
        else { ema_l = alpha*inst_l + (1.0f-alpha)*ema_l; ema_r = alpha*inst_r + (1.0f-alpha)*ema_r; }

        vel_l = ema_l; vel_r = ema_r;

        total_dist_l_mm += ticks_l * mm_per_tick;
        total_dist_r_mm += ticks_r * mm_per_tick;

        vel_l_start = cur_l; vel_r_start = cur_r; vel_window_start = now;
    }

    // ===== heading correction =====
    float heading_correction_mm_s = 0.0f;
    float current_heading = NAN;
    int16_t mx=0,my=0,mz=0;

    if (imu_ready && heading_locked && startup_state == STATE_RUNNING) {
        magnetometer_data_t md;
        if (magnetometer_read_data(&md)) {
            current_heading = md.heading;
            mx = md.x; my = md.y; mz = md.z;

            static float filt_err = 0.0f; static bool f_ok=false;
            float err = normalize_angle_error(target_heading - current_heading);
            if (!f_ok){ filt_err = err; f_ok = true; }
            else { filt_err = 0.3f*err + 0.7f*filt_err; }

            if (fabsf(filt_err) > HEADING_DEADZONE && speed_pct > 20) {
                heading_correction_mm_s = pid_update(&pid_heading, 0.0f, filt_err);
            } else {
                pid_heading.integ = 0.0f;
                f_ok = false;
            }
        }
    }

    // ===== wheel control =====
    float base = target_mm_s_from_ui();
    float target_l = base - heading_correction_mm_s;
    float target_r = base + heading_correction_mm_s;

    pid_l.dt_s = dt_s; pid_r.dt_s = dt_s;

    float ff_l = feedforward_left(target_l);
    float ff_r = feedforward_right(target_r);
    float pwm_l = clampf(ff_l + pid_update(&pid_l, target_l, vel_l), -100.0f, +100.0f);
    float pwm_r = clampf(ff_r + pid_update(&pid_r, target_r, vel_r), -100.0f, +100.0f);

    // slew
    static float prev_l=0.0f, prev_r=0.0f;
    const float MAX_STEP = 5.0f;
    float dl = clampf(pwm_l - prev_l, -MAX_STEP, +MAX_STEP);
    float dr = clampf(pwm_r - prev_r, -MAX_STEP, +MAX_STEP);
    pwm_l = prev_l + dl; pwm_r = prev_r + dr;
    prev_l = pwm_l; prev_r = pwm_r;

    motor_set_signed(pwm_r, pwm_l); // wiring dependent

    //For Telemetry ALWAYS KEEP IN FSM
    float cm = ultrasonic_measure_cm();
    int ultra_cm = (isnan(cm) ? -1 : (int)cm);

    uint16_t ir_raw = ir_line_read_adc_averaged(8);
    bool ir_on = ir_line_is_on_line();

    g_tm.ultra_cm    = ultra_cm;
    g_tm.ir_line_raw = ir_raw;
    g_tm.ir_on_line  = ir_on;
    g_tm.left_ticks  = (int32_t)encoder_left_count();
    g_tm.right_ticks = (int32_t)encoder_right_count();
    g_tm.v_l_mm_s    = vel_l;
    g_tm.v_r_mm_s    = vel_r;
    g_tm.dist_l_mm   = total_dist_l_mm;
    g_tm.dist_r_mm   = total_dist_r_mm;

    g_tm.heading_deg = current_heading;
    g_tm.mx = mx; g_tm.my = my; g_tm.mz = mz;

    g_tm.barcode_char = ch;
    g_tm.bars_count   = last_bars_count;

    //Print into console
    if (now - last_print_ms >= DISPLAY_INTERVAL_MS) {
        printf("[TM] ultra=%dcm | IR=%u/%d | vL=%.0f vR=%.0f | dL=%.0f dR=%.0f | hdg=%s%.1f | code=%c bars=%d\n",
            g_tm.ultra_cm, g_tm.ir_line_raw, (int)g_tm.ir_on_line,
            g_tm.v_l_mm_s, g_tm.v_r_mm_s, g_tm.dist_l_mm, g_tm.dist_r_mm,
            isnan(g_tm.heading_deg)?"(N/A)":"" , isnan(g_tm.heading_deg)?0.0f:g_tm.heading_deg,
            (g_tm.barcode_char? g_tm.barcode_char : '-'), g_tm.bars_count);
        last_print_ms = now;
    }
}

void fsm_get_telemetry(fsm_telemetry_t *out) {
    if (!out) return;
    *out = g_tm;
}
