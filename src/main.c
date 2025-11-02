#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "config.h"
#include "fsm.h"

int main(void) {
    stdio_init_all();
    
    // Initialize ADC for IR sensor
    adc_init();
    
    fsm_init();

    while (true) {
        fsm_step();
        tight_loop_contents();
    }
}