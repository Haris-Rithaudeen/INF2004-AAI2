#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "config.h"
#include "drivers/encoder.h"

static volatile uint32_t g_left_ticks  = 0;
static volatile uint32_t g_right_ticks = 0;

static inline void enc_gpio_init(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
#if ENCODER_PULLUP
    gpio_pull_up(pin);
#else
    gpio_pull_down(pin);
#endif
}

void encoder_on_gpio_irq(uint gpio, uint32_t events) {
    uint32_t mask = 0;
#if ENCODER_COUNT_BOTH_EDGES
    mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
#else
    mask = GPIO_IRQ_EDGE_RISE;
#endif
    if ((events & mask) == 0) return;

    if (gpio == LEFT_ENCODER_PIN)  { g_left_ticks++;  }
    if (gpio == RIGHT_ENCODER_PIN) { g_right_ticks++; }
}

#if ENCODER_OWNS_IRQ_CALLBACK
static void encoder_irq_global(uint gpio, uint32_t events) {
    encoder_on_gpio_irq(gpio, events);
}
#endif

void encoder_init(void) {
    enc_gpio_init(LEFT_ENCODER_PIN);
    enc_gpio_init(RIGHT_ENCODER_PIN);

#if ENCODER_OWNS_IRQ_CALLBACK
    // Register the global IRQ callback ourselves
#if ENCODER_COUNT_BOTH_EDGES
    const uint32_t edge_mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
#else
    const uint32_t edge_mask = GPIO_IRQ_EDGE_RISE;
#endif
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN,  edge_mask, true, &encoder_irq_global);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, edge_mask, true);
#else
    // Enable IRQs on pins only; app must call encoder_on_gpio_irq() from its global ISR
#if ENCODER_COUNT_BOTH_EDGES
    const uint32_t edge_mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
#else
    const uint32_t edge_mask = GPIO_IRQ_EDGE_RISE;
#endif
    gpio_set_irq_enabled(LEFT_ENCODER_PIN,  edge_mask, true);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, edge_mask, true);
#endif
}

void encoder_reset_counts(void) {
    g_left_ticks  = 0;
    g_right_ticks = 0;
}

uint32_t encoder_left_count(void)  { return g_left_ticks;  }
uint32_t encoder_right_count(void) { return g_right_ticks; }

float encoder_mm_per_tick(void) {
    const float wheel_circ_mm = (float)M_PI * WHEEL_DIAMETER_MM;
    return wheel_circ_mm / ENCODER_CPR;
}

float encoder_ticks_to_mm(uint32_t ticks) {
    return ticks * encoder_mm_per_tick();
}