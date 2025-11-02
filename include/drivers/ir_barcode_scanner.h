#ifndef IR_BARCODE_SCANNER_H
#define IR_BARCODE_SCANNER_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Bar structure for barcode detection
typedef struct {
    bool is_black;      // true = black bar, false = white space
    uint32_t width_us;  // Duration in microseconds
} barcode_bar_t;

// Barcode data structure
typedef struct {
    barcode_bar_t bars[50];  // Array of detected bars
    uint8_t bar_count;        // Number of bars detected
    bool scan_complete;       // true when full barcode read
    char decoded_value[32];   // Decoded barcode string
} barcode_data_t;

// Code 39 decoding structures
typedef struct {
    bool is_prev_black_bar;
    bool is_scanning;
    uint8_t delimiter_count;  // Count '*' delimiters
    uint64_t last_transition_time;
    uint8_t bar_index;
} barcode_flags_t;

// Initialize barcode scanner IR sensor
void ir_barcode_scanner_init(void);

// Start a new barcode scan
void ir_barcode_start_scan(void);

// Stop current barcode scan
void ir_barcode_stop_scan(void);

// Check if barcode scan is in progress
bool ir_barcode_is_scanning(void);

// Update barcode scan (call this in your control loop)
void ir_barcode_update(void);

// Get the latest barcode data
barcode_data_t* ir_barcode_get_data(void);

// Decode Code 39 barcode from bar timing arrays
char ir_barcode_decode_code39(int black_bar_times[5], int white_bar_times[5]);

// Full decode function - processes all bars and returns complete string
bool ir_barcode_decode(barcode_data_t* data);

// Print barcode data for debugging
void ir_barcode_print_data(void);

// Read raw ADC value
uint16_t ir_barcode_read_adc_raw(void);

// Reset barcode scanner state
void ir_barcode_reset(void);

// Get decoded character (for single character barcodes)
char ir_barcode_get_char(void);

// Check if a valid character has been decoded
bool ir_barcode_has_char(void);

// Clear the character ready flag
void ir_barcode_clear_char(void);

#endif