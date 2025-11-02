#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "config.h"
#include "drivers/motor.h"

static inline uint get_pwm_slice(uint gpio) { 
    return pwm_gpio_to_slice_num(gpio); 
}

void motor_init_all(void) {
    // Safe state low
    gpio_init(M1_A); gpio_set_dir(M1_A, GPIO_OUT); gpio_put(M1_A, 0);
    gpio_init(M1_B); gpio_set_dir(M1_B, GPIO_OUT); gpio_put(M1_B, 0);
    gpio_init(M2_A); gpio_set_dir(M2_A, GPIO_OUT); gpio_put(M2_A, 0);
    gpio_init(M2_B); gpio_set_dir(M2_B, GPIO_OUT); gpio_put(M2_B, 0);

    // Configure both PWM slices (GP8/9 share slice 4, GP10/11 share slice 5)
    uint slice1 = get_pwm_slice(M1_A);
    uint slice2 = get_pwm_slice(M2_A);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_CLKDIV_DEFAULT);
    pwm_init(slice1, &cfg, true);
    pwm_init(slice2, &cfg, true);

    pwm_set_wrap(slice1, PWM_WRAP_8BIT);
    pwm_set_wrap(slice2, PWM_WRAP_8BIT);
}

void motor_set_speed_percent(uint speed_percent, bool anticlockwise) {
    if (speed_percent > 100) speed_percent = 100;
    uint16_t level = (PWM_WRAP_8BIT * speed_percent) / 100;

    uint slice1 = get_pwm_slice(M1_A);  // Right motor
    uint slice2 = get_pwm_slice(M2_A);  // Left motor

    pwm_set_wrap(slice1, PWM_WRAP_8BIT);
    pwm_set_wrap(slice2, PWM_WRAP_8BIT);

    // Helper lambdas to apply inversion flags
    bool right_dir = anticlockwise ^ MOTOR_R_INVERT;
    bool left_dir  = anticlockwise ^ MOTOR_L_INVERT;

    // ---- RIGHT MOTOR (M1) ----
    if (right_dir) {
        gpio_set_function(M1_A, GPIO_FUNC_SIO); gpio_put(M1_A, 0);
        gpio_set_function(M1_B, GPIO_FUNC_PWM); pwm_set_chan_level(slice1, PWM_CHAN_B, level);
    } else {
        gpio_set_function(M1_B, GPIO_FUNC_SIO); gpio_put(M1_B, 0);
        gpio_set_function(M1_A, GPIO_FUNC_PWM); pwm_set_chan_level(slice1, PWM_CHAN_A, level);
    }

    // ---- LEFT MOTOR (M2) ----
    if (left_dir) {
        gpio_set_function(M2_A, GPIO_FUNC_SIO); gpio_put(M2_A, 0);
        gpio_set_function(M2_B, GPIO_FUNC_PWM); pwm_set_chan_level(slice2, PWM_CHAN_B, level);
    } else {
        gpio_set_function(M2_B, GPIO_FUNC_SIO); gpio_put(M2_B, 0);
        gpio_set_function(M2_A, GPIO_FUNC_PWM); pwm_set_chan_level(slice2, PWM_CHAN_A, level);
    }
}

void motor_stop(void) {
    gpio_set_function(M1_A, GPIO_FUNC_SIO); gpio_put(M1_A, 0);
    gpio_set_function(M1_B, GPIO_FUNC_SIO); gpio_put(M1_B, 0);
    gpio_set_function(M2_A, GPIO_FUNC_SIO); gpio_put(M2_A, 0);
    gpio_set_function(M2_B, GPIO_FUNC_SIO); gpio_put(M2_B, 0);
}

// factor out a tiny helper to set one motor given sign and inversion
static void drive_one(uint gpioA, uint gpioB, uint slice, float percent, bool invert_dir) {
    // clamp
    if (percent > 100.0f) percent = 100.0f;
    if (percent < -100.0f) percent = -100.0f;

    bool forward = (percent >= 0.0f);
    uint16_t level = (uint16_t)((PWM_WRAP_8BIT * (forward ? percent : -percent)) / 100.0f);

    // "Forward" can be flipped per-motor using invert_dir
    bool use_B_pwm = (forward ^ invert_dir); // true => B=PWM, A=LOW ; false => A=PWM, B=LOW

    if (use_B_pwm) {
        gpio_set_function(gpioA, GPIO_FUNC_SIO); gpio_put(gpioA, 0);
        gpio_set_function(gpioB, GPIO_FUNC_PWM);
        pwm_set_chan_level(slice, (gpioB == M1_B || gpioB == M2_B) ? PWM_CHAN_B : PWM_CHAN_A, level);
    } else {
        gpio_set_function(gpioB, GPIO_FUNC_SIO); gpio_put(gpioB, 0);
        gpio_set_function(gpioA, GPIO_FUNC_PWM);
        pwm_set_chan_level(slice, (gpioA == M1_A || gpioA == M2_A) ? PWM_CHAN_A : PWM_CHAN_B, level);
    }
}

void motor_set_signed(float right_percent, float left_percent) {
    uint slice_right = get_pwm_slice(M1_A);
    uint slice_left  = get_pwm_slice(M2_A);

    // keep wrap consistent
    pwm_set_wrap(slice_right, PWM_WRAP_8BIT);
    pwm_set_wrap(slice_left,  PWM_WRAP_8BIT);

    // RIGHT motor uses MOTOR_R_INVERT to flip "forward" sense
    drive_one(M1_A, M1_B, slice_right, right_percent, MOTOR_R_INVERT != 0);
    // LEFT motor uses MOTOR_L_INVERT
    drive_one(M2_A, M2_B, slice_left,  left_percent,  MOTOR_L_INVERT != 0);
}