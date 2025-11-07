#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "config/config.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/ir_line_follower.h"
#include "drivers/ir_barcode_scanner.h"
#include "drivers/ultrasonic.h"
#include "drivers/magnetometer.h"
#include "networking/wifi_mqtt.h"  // ADD THIS
#include "fsm.h"

static encoder_t encL, encR;
static float left_cmd = 0.30f, right_cmd = 0.30f;
static uint32_t stall_timer_ms_L = 0, stall_timer_ms_R = 0;
static float left_distance_mm = 0.0f, right_distance_mm = 0.0f;
static bool mqtt_enabled = false;  // ADD THIS

void fsm_init(void) {
    motor_init_all();
    encoder_init(&encL, PIN_ENC_LEFT,  ENCODER_PULLUP);
    encoder_init(&encR, PIN_ENC_RIGHT, ENCODER_PULLUP);
    
    // Initialize ADC once before both IR sensors
    adc_init();
    ir_line_follower_init();
    ir_barcode_scanner_init();
    
    // Initialize ultrasonic sensor
    ultrasonic_init();
    
    // Initialize magnetometer
    if (!magnetometer_init()) {
        printf("WARNING: Magnetometer initialization failed\n");
    }

    // Initialize WiFi and MQTT (ADD THIS SECTION)
    printf("[MQTT] Initializing WiFi...\n");
    if (wifi_mqtt_init()) {
        if (wifi_mqtt_connect()) {
            if (mqtt_connect_broker()) {
                mqtt_enabled = true;
                printf("[MQTT] ✓ System ready\n");
            }
        }
    }
    if (!mqtt_enabled) {
        printf("[MQTT] × MQTT disabled, continuing without networking\n");
    }

    // Buttons
    gpio_init(PIN_BTN_DIR); gpio_set_dir(PIN_BTN_DIR, GPIO_IN); gpio_pull_up(PIN_BTN_DIR);
    gpio_init(PIN_BTN_SPD); gpio_set_dir(PIN_BTN_SPD, GPIO_IN); gpio_pull_up(PIN_BTN_SPD);

    printf("FSM demo: 100 Hz | CPR=%.0f | wheel=%.1f mm\n", ENCODER_CPR, WHEEL_DIAMETER_MM);
}

void fsm_step(void) {
    static bool last_dir = true, last_spd = true;
    static uint32_t last_dir_t = 0, last_spd_t = 0;
    static uint32_t last_print_ms = 0;
    static uint32_t last_line_print_ms = 0;
    static uint32_t last_barcode_print_ms = 0;
    static uint32_t last_ultrasonic_print_ms = 0;
    static uint32_t last_mag_print_ms = 0;
    static uint32_t last_mqtt_publish_ms = 0;  // ADD THIS

    const float wheel_circ_mm = (float)M_PI * WHEEL_DIAMETER_MM;
    const float mm_per_tick   = wheel_circ_mm / ENCODER_CPR;

    bool dir_now = gpio_get(PIN_BTN_DIR);
    bool spd_now = gpio_get(PIN_BTN_SPD);
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // Poll WiFi/MQTT (ADD THIS)
    if (mqtt_enabled) {
        wifi_mqtt_poll();
    }

    // Button handling (same as before)
    if (!dir_now && last_dir && (now_ms - last_dir_t > 200)) {
        left_cmd  = -left_cmd;
        right_cmd = -right_cmd;
        last_dir_t = now_ms;
    }
    if (!spd_now && last_spd && (now_ms - last_spd_t > 200)) {
        float s = fabsf(left_cmd);
        s = (s < 0.35f) ? 0.60f : (s < 0.80f ? 0.90f : 0.30f);
        left_cmd  = copysignf(s, left_cmd);
        right_cmd = copysignf(s, right_cmd);
        last_spd_t = now_ms;
        
        if (!ir_barcode_is_scanning()) {
            ir_barcode_start_scan();
        }
    }
    last_dir = dir_now; last_spd = spd_now;

    // Obstacle detection
    float obstacle_distance = ultrasonic_measure_cm();
    if (!isnan(obstacle_distance) && obstacle_distance < 20.0f) {
        printf("[OBSTACLE DETECTED] Distance: %.2f cm - STOPPING\n", obstacle_distance);
        motor_stop();
        left_cmd = right_cmd = 0.0f;
    } else {
        motor_set(left_cmd, right_cmd);
    }

    // Encoder reading
    uint32_t dticks_L = encoder_read_and_clear(&encL);
    uint32_t dticks_R = encoder_read_and_clear(&encR);

    float dmm_L = dticks_L * mm_per_tick;
    float dmm_R = dticks_R * mm_per_tick;
    left_distance_mm  += dmm_L;
    right_distance_mm += dmm_R;

    float v_mm_s_L = dmm_L * (1000.0f / CONTROL_DT_MS);
    float v_mm_s_R = dmm_R * (1000.0f / CONTROL_DT_MS);

    // Stall detection (same as before)
    if (fabsf(left_cmd) > STALL_PWM_THRESHOLD) {
        stall_timer_ms_L = (dticks_L == 0) ? (stall_timer_ms_L + CONTROL_DT_MS) : 0;
        if (stall_timer_ms_L >= STALL_WINDOW_MS) {
            printf("[FAULT] Left motor stall, stopping.\n");
            motor_stop();
            left_cmd = right_cmd = 0.0f;
        }
    } else { stall_timer_ms_L = 0; }

    if (fabsf(right_cmd) > STALL_PWM_THRESHOLD) {
        stall_timer_ms_R = (dticks_R == 0) ? (stall_timer_ms_R + CONTROL_DT_MS) : 0;
        if (stall_timer_ms_R >= STALL_WINDOW_MS) {
            printf("[FAULT] Right motor stall, stopping.\n");
            motor_stop();
            left_cmd = right_cmd = 0.0f;
        }
    } else { stall_timer_ms_R = 0; }

    ir_barcode_update();

    // Telemetry printing (same as before)
    if (now_ms - last_print_ms >= 500) {
        printf("L: v=%.1fmm/s dist=%.0f | R: v=%.1fmm/s dist=%.0f\n",
               v_mm_s_L, left_distance_mm, v_mm_s_R, right_distance_mm);
        last_print_ms = now_ms;
    }

    if (now_ms - last_line_print_ms >= IR_LINE_PRINT_INTERVAL_MS) {
        ir_line_print_data();
        last_line_print_ms = now_ms;
    }

    if (now_ms - last_barcode_print_ms >= 2000) {
        if (ir_barcode_is_scanning() || ir_barcode_get_data()->bar_count > 0) {
            ir_barcode_print_data();
        }
        last_barcode_print_ms = now_ms;
    }

    if (now_ms - last_ultrasonic_print_ms >= ULTRASONIC_PRINT_INTERVAL_MS) {
        ultrasonic_print_data();
        last_ultrasonic_print_ms = now_ms;
    }

    if (now_ms - last_mag_print_ms >= MAG_PRINT_INTERVAL_MS) {
        magnetometer_print_data();
        last_mag_print_ms = now_ms;
    }

    // MQTT telemetry publishing (ADD THIS SECTION)
    if (mqtt_enabled && mqtt_is_connected() && (now_ms - last_mqtt_publish_ms >= 1000)) {
        magnetometer_data_t mag_data;
        magnetometer_read_data(&mag_data);
        
        mqtt_publish_telemetry(
            v_mm_s_L / 10.0f,           // Convert mm/s to cm/s
            v_mm_s_R / 10.0f,
            left_distance_mm / 10.0f,   // Convert mm to cm
            right_distance_mm / 10.0f,
            mag_data.heading,
            mag_data.x, mag_data.y, mag_data.z
        );
        // Publish ultrasonic + IR line sensor data
        uint16_t line_raw = ir_line_read_adc_raw();
        bool on_line = ir_line_is_on_line();
        mqtt_publish_sensors(
            obstacle_distance,  // Already measured above
            line_raw,
            on_line
        );
        
        // Publish barcode data if available
        barcode_data_t* barcode = ir_barcode_get_data();
        if (barcode->scan_complete && barcode->bar_count > 0) {
            mqtt_publish_barcode(
                barcode->decoded_value,
                barcode->bar_count
            );
        }
        
        last_mqtt_publish_ms = now_ms;
    }
}