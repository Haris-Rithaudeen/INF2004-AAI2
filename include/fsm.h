#pragma once
#include <stdint.h>
#include <stdbool.h>

// ----- Public telemetry struct (filled by FSM) -----
typedef struct {
    // Ultrasonic
    int   ultra_cm;

    // IR line follower
    uint16_t ir_line_raw;
    bool     ir_on_line;

    // Encoders / odom
    int32_t left_ticks;
    int32_t right_ticks;
    float   v_l_mm_s;
    float   v_r_mm_s;
    float   dist_l_mm;
    float   dist_r_mm;

    // Magnetometer / heading
    float   heading_deg;   // NAN if not available
    int16_t mx, my, mz;    // smoothed raw axes

    //Barcode scanner (might need updating)
    char    barcode_char;  
    int     bars_count;    
} fsm_telemetry_t;

void fsm_init(void);
void fsm_step(uint32_t now_ms);
void fsm_get_telemetry(fsm_telemetry_t *out);
//handle inbound MQTT commands
void fsm_on_cmd(const char *topic, const uint8_t *payload, size_t len);

