#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    //Ultrasonic
    int   ultra_cm;

    //IR line follower
    uint16_t ir_line_raw;
    bool     ir_on_line;

    //Speed and Distance
    int32_t left_ticks;
    int32_t right_ticks;
    float   v_l_mm_s;
    float   v_r_mm_s;
    float   dist_l_mm;
    float   dist_r_mm;

    //Magnetometer
    float   heading_deg;
    int16_t mx, my, mz;

    //Barcode
    char    barcode_char;
    int     bars_count;
} fsm_telemetry_t;

void fsm_init(void);
void fsm_step(uint32_t now_ms);
void fsm_get_telemetry(fsm_telemetry_t *out);

void fsm_on_cmd(const char *topic, const uint8_t *payload, size_t len);
