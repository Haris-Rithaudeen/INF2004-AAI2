#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "config.h"
#include "drivers/ir_line_follower.h"
#include "fsm.h"

void fsm_init(void) {
    // Initialize ADC once before IR sensor
    adc_init();
    ir_line_follower_init();

    printf("FSM demo: IR Line Follower only\n");
}

void fsm_step(void) {
    static uint32_t last_line_print_ms = 0;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // Telemetry printing
    if (now_ms - last_line_print_ms >= IR_LINE_PRINT_INTERVAL_MS) {
        ir_line_print_data();
        last_line_print_ms = now_ms;
    }
}