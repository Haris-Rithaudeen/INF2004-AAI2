
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "config/config.h"
#include "drivers/ultrasonic.h"
#include "fsm.h"

// Minimal FSM to test ONLY the Ultrasonic sensor extracted from combfsm.c
// - Initializes ultrasonic on boot
// - Periodically prints distance readings
// - No motors, no encoders, no IR, no magnetometer, no WiFi/MQTT

static uint32_t last_print_ms = 0;

static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

void fsm_init(void) {
    stdio_init_all();
    ultrasonic_init();
    printf("\n=== COMBFSM: ULTRASONIC-ONLY TEST MODE ===\n");
    printf("TRIG=GP%d, ECHO=GP%d\n", ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO);
    printf("Print interval: %d ms\n", ULTRASONIC_PRINT_INTERVAL_MS);
}

void fsm_step(void) {
    uint32_t t = now_ms();
    if (t - last_print_ms >= (uint32_t)ULTRASONIC_PRINT_INTERVAL_MS) {
        float cm = ultrasonic_measure_cm();
        if (isnan(cm)) {
            printf("[ULTRASONIC] Out of range / timeout\n");
        } else {
            printf("[ULTRASONIC] Distance: %.2f cm\n", cm);
        }
        last_print_ms = t;
    }
}
