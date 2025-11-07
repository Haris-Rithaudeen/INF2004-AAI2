#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "config/config.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "fsm.h"

// ---- Unified GPIO ISR for buttons + encoders ----
static volatile bool g_toggle_dir_req = false;
static volatile bool g_change_speed_req = false;
static volatile uint32_t g_last_dir_ms = 0;
static volatile uint32_t g_last_spd_ms = 0;

static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void gpio_isr_unified(uint gpio, uint32_t events) {
    // Forward event to encoder module (increments ticks if from encoder pins)
    encoder_on_gpio_irq(gpio, events);

    // Handle buttons (active-low: use falling edge)
    uint32_t t = now_ms();
    if (gpio == BUTTON_DIR && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_dir_ms) > DEBOUNCE_MS) { g_toggle_dir_req = true; g_last_dir_ms = t; }
    }
    if (gpio == BUTTON_SPD && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_spd_ms) > DEBOUNCE_MS) { g_change_speed_req = true; g_last_spd_ms = t; }
    }
}

// ---- App state ----
static bool anticlockwise = false;
static uint speed = 70;
static uint32_t last_print_ms = 0;

void fsm_init(void) {
    stdio_init_all();
    motor_init_all();

    // Encoders
    encoder_init(); // sets GPIO directions and enables IRQs on pins (no callback)

    // Buttons (active-low)
    gpio_init(BUTTON_DIR); gpio_set_dir(BUTTON_DIR, GPIO_IN); gpio_pull_up(BUTTON_DIR);
    gpio_init(BUTTON_SPD); gpio_set_dir(BUTTON_SPD, GPIO_IN); gpio_pull_up(BUTTON_SPD);

    // Register one ISR for both buttons and encoders
    const uint32_t btn_edge = GPIO_IRQ_EDGE_FALL;
#if ENCODER_COUNT_BOTH_EDGES
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
#else
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE;
#endif
    // Enable events on all pins we care about, bind the unified callback
    gpio_set_irq_enabled_with_callback(BUTTON_DIR, btn_edge, true, &gpio_isr_unified);
    gpio_set_irq_enabled(BUTTON_SPD, btn_edge, true);
    gpio_set_irq_enabled(LEFT_ENCODER_PIN,  enc_edge, true);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, enc_edge, true);

    // Start motors
    motor_set_speed_percent(speed, anticlockwise);

    printf("FSM (interrupts for buttons + H206 encoders) ready.\n");
    printf("GP21: toggle direction | GP20: randomize 40..100%%\n");
    printf("Encoders on GPIO IRQs | CPR=%.0f | wheel=%.1fmm | mm/tick=%.2f\n",
           ENCODER_CPR, WHEEL_DIAMETER_MM, encoder_mm_per_tick());
}

void fsm_step(void) {
    if (g_toggle_dir_req) {
        g_toggle_dir_req = false;
        anticlockwise = !anticlockwise;
        motor_set_speed_percent(speed, anticlockwise);
        printf("Direction: %s\n", anticlockwise ? "Anticlockwise" : "Clockwise");
    }
    if (g_change_speed_req) {
        g_change_speed_req = false;
        speed = 40 + (rand() % 61);
        motor_set_speed_percent(speed, anticlockwise);
        printf("Speed: %u%%\n", speed);
    }

    uint32_t now = now_ms();
    if (now - last_print_ms >= DISPLAY_INTERVAL_MS) {
        uint32_t lt = encoder_left_count();
        uint32_t rt = encoder_right_count();
        float lmm = encoder_ticks_to_mm(lt);
        float rmm = encoder_ticks_to_mm(rt);
        printf("--- Status ---\n");
        printf("Speed: %u%% | Dir: %s\n", speed, anticlockwise ? "Anticlockwise" : "Clockwise");
        printf("L: ticks=%lu dist=%.0fmm | R: ticks=%lu dist=%.0fmm\n",
               (unsigned long)lt, lmm, (unsigned long)rt, rmm);
        encoder_reset_counts();
        last_print_ms = now;
    }
}
