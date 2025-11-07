#ifndef DRIVERS_ENCODER_H
#define DRIVERS_ENCODER_H
#pragma once
#include <stdint.h>
#include <stdbool.h>

// H206 single-channel encoder driver (interrupt-based)
// Disc: ENCODER_CPR ticks/rev (config.h), wheel diameter WHEEL_DIAMETER_MM (config.h).
// We count **rising edges** by default. If you want both edges, define ENCODER_COUNT_BOTH_EDGES 1.
//
// API:
//   void  encoder_init(void);                      // sets up GPIOs and IRQs
//   void  encoder_reset_counts(void);              // zero accumulated tick counters
//   uint32_t encoder_left_count(void);
//   uint32_t encoder_right_count(void);
//   float encoder_mm_per_tick(void);               // geometric conversion helper
//   float encoder_ticks_to_mm(uint32_t ticks);     // ticks -> mm
//
// Integration with a global GPIO ISR:
//   If your app already owns the global GPIO IRQ callback (e.g., for buttons),
//   call encoder_on_gpio_irq(gpio, events) from that ISR so encoders increment.
//
//   Alternatively, define ENCODER_OWNS_IRQ_CALLBACK 1 before including this header
//   (or in config.h) to let the encoder module register the global callback itself.

void encoder_init(void);
void encoder_reset_counts(void);
uint32_t encoder_left_count(void);
uint32_t encoder_right_count(void);
float encoder_mm_per_tick(void);
float encoder_ticks_to_mm(uint32_t ticks);
void encoder_on_gpio_irq(uint gpio, uint32_t events);

#endif