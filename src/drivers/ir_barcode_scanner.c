#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "config.h"
#include "drivers/ir_barcode_scanner.h"

// Code 39 character lookup table
static const char code_39_characters[] = "1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *";

static barcode_data_t barcode_data;
static barcode_flags_t barcode_flags;
static bool scanning_active = false;
static uint64_t last_transition_us = 0;
static bool last_state = false;

// For Code 39 decoding
static int black_bar_times[5] = {0};
static int white_bar_times[5] = {0};
static char decoded_char = '\0';
static bool char_ready = false;

void ir_barcode_scanner_init(void) {
    // Initialize ADC if not already done
    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_initialized = true;
    }
    
    adc_gpio_init(IR_BARCODE_ADC_GPIO);
    
    gpio_init(IR_BARCODE_DIGITAL_GPIO);
    gpio_set_dir(IR_BARCODE_DIGITAL_GPIO, GPIO_IN);
    gpio_disable_pulls(IR_BARCODE_DIGITAL_GPIO);

    memset(&barcode_data, 0, sizeof(barcode_data_t));
    memset(&barcode_flags, 0, sizeof(barcode_flags_t));
    
    printf("Barcode Scanner IR initialized: ADC on GP%d (ch%d), Digital on GP%d\n",
           IR_BARCODE_ADC_GPIO, IR_BARCODE_ADC_CHANNEL, IR_BARCODE_DIGITAL_GPIO);
}

void ir_barcode_start_scan(void) {
    memset(&barcode_data, 0, sizeof(barcode_data_t));
    memset(&barcode_flags, 0, sizeof(barcode_flags_t));
    memset(black_bar_times, 0, sizeof(black_bar_times));
    memset(white_bar_times, 0, sizeof(white_bar_times));
    
    barcode_flags.is_scanning = true;
    scanning_active = true;
    barcode_flags.last_transition_time = time_us_64();
    last_state = gpio_get(IR_BARCODE_DIGITAL_GPIO);
    decoded_char = '\0';
    char_ready = false;
    
    printf("[BARCODE] Scan started\n");
}

void ir_barcode_stop_scan(void) {
    scanning_active = false;
    barcode_flags.is_scanning = false;
    printf("[BARCODE] Scan stopped, detected %d bars\n", barcode_data.bar_count);
}

void ir_barcode_reset(void) {
    memset(&barcode_data, 0, sizeof(barcode_data_t));
    memset(&barcode_flags, 0, sizeof(barcode_flags_t));
    memset(black_bar_times, 0, sizeof(black_bar_times));
    memset(white_bar_times, 0, sizeof(white_bar_times));
    decoded_char = '\0';
    char_ready = false;
    scanning_active = false;
}

bool ir_barcode_is_scanning(void) {
    return scanning_active;
}

// Decode single Code 39 character from bar timings (based on Ryan's implementation)
char ir_barcode_decode_code39(int black_bar_times[5], int white_bar_times[5]) {
    // Calculate relative timings
    int dec_black_bar_times[5];
    int dec_white_bar_times[4];
    
    dec_black_bar_times[0] = (white_bar_times[0] - black_bar_times[0]);
    dec_black_bar_times[1] = (white_bar_times[1] - black_bar_times[1]);
    dec_black_bar_times[2] = (white_bar_times[2] - black_bar_times[2]);
    dec_black_bar_times[3] = (white_bar_times[3] - black_bar_times[3]);
    dec_black_bar_times[4] = (white_bar_times[4] - black_bar_times[4]);

    dec_white_bar_times[0] = (black_bar_times[1] - white_bar_times[0]);
    dec_white_bar_times[1] = (black_bar_times[2] - white_bar_times[1]);
    dec_white_bar_times[2] = (black_bar_times[3] - white_bar_times[2]);
    dec_white_bar_times[3] = (black_bar_times[4] - white_bar_times[3]);

    // Find two highest black bars
    int max1 = 0, max2 = 0;
    for (int i = 0; i < 5; i++) {
        if (dec_black_bar_times[i] > max1) {
            max2 = max1;
            max1 = dec_black_bar_times[i];
        } else if (dec_black_bar_times[i] > max2) {
            max2 = dec_black_bar_times[i];
        }
    }

    // Set the two highest to 1, rest to 0 for black bars
    for (int i = 0; i < 5; i++) {
        if (dec_black_bar_times[i] == max1 || dec_black_bar_times[i] == max2) {
            dec_black_bar_times[i] = 1;
        } else {
            dec_black_bar_times[i] = 0;
        }
    }

    // Find highest white bar
    int max_white = dec_white_bar_times[0];
    for (int i = 1; i < 4; i++) {
        if (dec_white_bar_times[i] > max_white) {
            max_white = dec_white_bar_times[i];
        }
    }

    // Set highest to 1, rest to 0 for white bars
    for (int i = 0; i < 4; i++) {
        if (dec_white_bar_times[i] == max_white) {
            dec_white_bar_times[i] = 1;
        } else {
            dec_white_bar_times[i] = 0;
        }
    }

    // Calculate result index based on Code 39 encoding
    int result = 0;

    // Black bar encoding
    if (dec_black_bar_times[0] && dec_black_bar_times[4])
        result += 1;
    else if (dec_black_bar_times[1] && dec_black_bar_times[4])
        result += 2;
    else if (dec_black_bar_times[0] && dec_black_bar_times[1])
        result += 3;
    else if (dec_black_bar_times[2] && dec_black_bar_times[4])
        result += 4;
    else if (dec_black_bar_times[0] && dec_black_bar_times[2])
        result += 5;
    else if (dec_black_bar_times[1] && dec_black_bar_times[2])
        result += 6;
    else if (dec_black_bar_times[3] && dec_black_bar_times[4])
        result += 7;
    else if (dec_black_bar_times[0] && dec_black_bar_times[3])
        result += 8;
    else if (dec_black_bar_times[1] && dec_black_bar_times[3])
        result += 9;
    else if (dec_black_bar_times[2] && dec_black_bar_times[3])
        result += 10;

    // White bar encoding
    if (dec_white_bar_times[1])
        result += 0;
    else if (dec_white_bar_times[2])
        result += 9;
    else if (dec_white_bar_times[3])
        result += 19;
    else if (dec_white_bar_times[0])
        result += 29;
    
    if (result >= 0 && result < 40) {
        return code_39_characters[result];
    }
    
    return '\0';
}

void ir_barcode_update(void) {
    if (!scanning_active) return;
    
    // Read ADC value
    uint16_t reading = ir_barcode_read_adc_raw();
    uint64_t now_us = time_us_64();
    
    // Detect black bar (above threshold)
    if (reading > IR_BARCODE_THRESHOLD && !barcode_flags.is_prev_black_bar) {
        // Transition to black bar
        barcode_flags.is_prev_black_bar = true;
        int timing = (int)(now_us - barcode_flags.last_transition_time);
        black_bar_times[barcode_flags.bar_index] = timing;
        barcode_flags.last_transition_time = now_us;
    }
    // Detect white space (below threshold)
    else if (reading < IR_BARCODE_THRESHOLD && barcode_flags.is_prev_black_bar) {
        // Transition to white space
        barcode_flags.is_prev_black_bar = false;
        int timing = (int)(now_us - barcode_flags.last_transition_time);
        white_bar_times[barcode_flags.bar_index] = timing;
        barcode_flags.last_transition_time = now_us;
        barcode_flags.bar_index++;
    }
    
    // Check if we've collected all 5 bars (one complete character)
    if (white_bar_times[4] != 0) {
        // Decode the character
        char ch = ir_barcode_decode_code39(black_bar_times, white_bar_times);
        
        if (ch == '*') {
            // Delimiter detected
            barcode_flags.delimiter_count++;
            printf("[BARCODE] Delimiter '*' detected (count: %d)\n", barcode_flags.delimiter_count);
            
            // If we've seen 2 delimiters, scanning complete
            if (barcode_flags.delimiter_count >= 2) {
                printf("[BARCODE] Scan complete!\n");
                barcode_data.scan_complete = true;
                scanning_active = false;
                barcode_flags.is_scanning = false;
            }
        } else if (ch != '\0') {
            // Valid character decoded
            decoded_char = ch;
            char_ready = true;
            printf("[BARCODE] ✓ Character decoded: '%c'\n", decoded_char);
            
            // Store in decoded string
            int len = strlen(barcode_data.decoded_value);
            if (len < sizeof(barcode_data.decoded_value) - 1) {
                barcode_data.decoded_value[len] = ch;
                barcode_data.decoded_value[len + 1] = '\0';
            }
        }
        
        // Reset for next character
        memset(black_bar_times, 0, sizeof(black_bar_times));
        memset(white_bar_times, 0, sizeof(white_bar_times));
        barcode_flags.bar_index = 0;
        barcode_flags.is_prev_black_bar = false;
    }
}

barcode_data_t* ir_barcode_get_data(void) {
    return &barcode_data;
}

bool ir_barcode_decode(barcode_data_t* data) {
    // Already decoded in update loop
    return (strlen(data->decoded_value) > 0);
}

void ir_barcode_print_data(void) {
    printf("[BARCODE] Decoded string: \"%s\"\n", barcode_data.decoded_value);
    printf("[BARCODE] Last character: '%c'\n", decoded_char);
    printf("[BARCODE] Scan complete: %s\n", barcode_data.scan_complete ? "YES" : "NO");
}

uint16_t ir_barcode_read_adc_raw(void) {
    adc_select_input(IR_BARCODE_ADC_CHANNEL);
    return adc_read();
}

char ir_barcode_get_char(void) {
    return decoded_char;
}

bool ir_barcode_has_char(void) {
    return char_ready;
}

void ir_barcode_clear_char(void) {
    char_ready = false;
    decoded_char = '\0';
}