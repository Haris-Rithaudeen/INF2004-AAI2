
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "config.h"
#include "drivers/magnetometer.h"
#include "fsm.h"

// Minimal FSM to test ONLY the Magnetometer (LSM303) extracted from combfsm.c
// - Initializes I2C + magnetometer with SMOOTH MOVING AVERAGE filter
// - Periodically reads raw XYZ and heading, and prints them
// - Shows both FILTERED (smooth) and RAW (unfiltered) readings for comparison
// - No motors, no encoders, no IR, no ultrasonic, no WiFi/MQTT

static uint32_t last_print_ms = 0;

static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

void fsm_init(void) {
    stdio_init_all();

    // Initialize magnetometer (LSM303)
    if (magnetometer_init()) {
        printf("\n=== COMBFSM: MAGNETOMETER TEST MODE (WITH SMOOTH MOVING AVERAGE) ===\n");
        printf("I2C: SDA=GP%d, SCL=GP%d, LSM303_MAG_ADDR=0x%02X\n",
               I2C_SDA_PIN, I2C_SCL_PIN, LSM303_MAG_ADDR);
        printf("Print interval: %d ms\n", MAG_PRINT_INTERVAL_MS);
        printf("Filter size: %d samples (moving average)\n", MAG_FILTER_SIZE);
        printf("Rotate the board slowly to see heading change (0°=N, 90°=E, 180°=S, 270°=W)\n");
        printf("First %d readings will show partial averaging as filter fills up\n\n", MAG_FILTER_SIZE);
    } else {
        printf("[MAG] Initialization FAILED. Check wiring and I2C pins in config.h\n");
    }
}

void fsm_step(void) {
    uint32_t t = now_ms();
    if (t - last_print_ms >= (uint32_t)MAG_PRINT_INTERVAL_MS) {
        magnetometer_data_t filtered, raw;
        
        // Read filtered (smoothed) data
        bool filtered_ok = magnetometer_read_data(&filtered);
        
        // Read raw (unfiltered) data
        bool raw_ok = magnetometer_read_data_raw(&raw);
        
        if (filtered_ok && raw_ok) {
            // Print both filtered and raw values for comparison
            printf("[MAG SMOOTH] x=%6d y=%6d z=%6d | heading=%6.1f°", 
                   filtered.x, filtered.y, filtered.z, filtered.heading);
            
            printf("  [RAW] x=%6d y=%6d z=%6d | heading=%6.1f°\n", 
                   raw.x, raw.y, raw.z, raw.heading);
        } else {
            printf("[MAG] read failed\n");
        }
        last_print_ms = t;
    }
}