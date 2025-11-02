#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "config.h"
#include "drivers/magnetometer.h"

// LSM303 Magnetometer Registers
#define LSM303_REG_CRA_M        0x00    // Control Register A
#define LSM303_REG_CRB_M        0x01    // Control Register B
#define LSM303_REG_MR_M         0x02    // Mode Register
#define LSM303_REG_OUT_X_H_M    0x03    // X-axis MSB
#define LSM303_REG_OUT_X_L_M    0x04    // X-axis LSB
#define LSM303_REG_OUT_Z_H_M    0x05    // Z-axis MSB (note: Z before Y!)
#define LSM303_REG_OUT_Z_L_M    0x06    // Z-axis LSB
#define LSM303_REG_OUT_Y_H_M    0x07    // Y-axis MSB
#define LSM303_REG_OUT_Y_L_M    0x08    // Y-axis LSB
#define LSM303_REG_SR_M         0x09    // Status Register
#define LSM303_REG_IRA_M        0x0A    // Identification Register A
#define LSM303_REG_IRB_M        0x0B    // Identification Register B
#define LSM303_REG_IRC_M        0x0C    // Identification Register C

// Calibration data
static int16_t mag_min_x = 32767, mag_max_x = -32768;
static int16_t mag_min_y = 32767, mag_max_y = -32768;
static int16_t mag_min_z = 32767, mag_max_z = -32768;
static bool calibration_active = false;

// Moving average filter buffers
static int16_t filter_x[MAG_FILTER_SIZE] = {0};
static int16_t filter_y[MAG_FILTER_SIZE] = {0};
static int16_t filter_z[MAG_FILTER_SIZE] = {0};
static uint8_t filter_index = 0;
static uint8_t filter_count = 0;  // Tracks how many samples we've collected
static bool filter_initialized = false;

// Write to magnetometer register
static bool mag_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    int result = i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, buf, 2, false);
    return (result == 2);
}

// Read from magnetometer register
static bool mag_read(uint8_t reg, uint8_t *dst, size_t n) {
    // Write register address
    if (i2c_write_blocking(I2C_PORT, LSM303_MAG_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    // Read data
    return (i2c_read_blocking(I2C_PORT, LSM303_MAG_ADDR, dst, n, false) == (int)n);
}

// Apply moving average filter to the raw data
static void apply_moving_average(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                                  int16_t *smooth_x, int16_t *smooth_y, int16_t *smooth_z) {
    // Add new sample to circular buffer
    filter_x[filter_index] = raw_x;
    filter_y[filter_index] = raw_y;
    filter_z[filter_index] = raw_z;
    
    // Increment index and wrap around
    filter_index = (filter_index + 1) % MAG_FILTER_SIZE;
    
    // Track how many samples we have (up to MAG_FILTER_SIZE)
    if (filter_count < MAG_FILTER_SIZE) {
        filter_count++;
    }
    
    // Calculate average of all samples in buffer
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    for (uint8_t i = 0; i < filter_count; i++) {
        sum_x += filter_x[i];
        sum_y += filter_y[i];
        sum_z += filter_z[i];
    }
    
    *smooth_x = (int16_t)(sum_x / filter_count);
    *smooth_y = (int16_t)(sum_y / filter_count);
    *smooth_z = (int16_t)(sum_z / filter_count);
    
    filter_initialized = true;
}

bool magnetometer_init(void) {
    // Initialize I2C
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    sleep_ms(10);  // Give sensor time to power up

    // Check magnetometer ID (optional)
    uint8_t id_a, id_b, id_c;
    if (mag_read(LSM303_REG_IRA_M, &id_a, 1) &&
        mag_read(LSM303_REG_IRB_M, &id_b, 1) &&
        mag_read(LSM303_REG_IRC_M, &id_c, 1)) {
        // Expected: 0x48, 0x34, 0x33 for LSM303DLHC
        if (id_a != 0x48 || id_b != 0x34 || id_c != 0x33) {
            printf("Warning: Magnetometer ID mismatch (0x%02X 0x%02X 0x%02X)\n", 
                   id_a, id_b, id_c);
        }
    }

    // Configure magnetometer
    // CRA_REG_M: Data output rate = 15 Hz
    if (!mag_write(LSM303_REG_CRA_M, 0x10)) {
        printf("Failed to write CRA_M\n");
        return false;
    }

    // CRB_REG_M: Gain = ±1.3 gauss
    if (!mag_write(LSM303_REG_CRB_M, 0x20)) {
        printf("Failed to write CRB_M\n");
        return false;
    }

    // MR_REG_M: Continuous conversion mode
    if (!mag_write(LSM303_REG_MR_M, 0x00)) {
        printf("Failed to write MR_M\n");
        return false;
    }

    sleep_ms(10);

    // Reset the moving average filter
    magnetometer_reset_filter();

    printf("Magnetometer initialized: I2C%d, SDA=GP%d, SCL=GP%d\n",
           i2c_hw_index(I2C_PORT), I2C_SDA_PIN, I2C_SCL_PIN);
    printf("Moving average filter: %d samples\n", MAG_FILTER_SIZE);

    return true;
}

void magnetometer_reset_filter(void) {
    for (uint8_t i = 0; i < MAG_FILTER_SIZE; i++) {
        filter_x[i] = 0;
        filter_y[i] = 0;
        filter_z[i] = 0;
    }
    filter_index = 0;
    filter_count = 0;
    filter_initialized = false;
}

bool magnetometer_read_raw(int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t buf[6];

    // Read all 6 bytes (X, Z, Y order in LSM303!)
    if (!mag_read(LSM303_REG_OUT_X_H_M, buf, 6)) {
        return false;
    }

    // Note: LSM303 outputs in X-Z-Y order (not X-Y-Z)
    int16_t x = (int16_t)((buf[0] << 8) | buf[1]);  // X
    int16_t z = (int16_t)((buf[2] << 8) | buf[3]);  // Z
    int16_t y = (int16_t)((buf[4] << 8) | buf[5]);  // Y

    if (mx) *mx = x;
    if (my) *my = y;
    if (mz) *mz = z;

    // Update calibration if active
    if (calibration_active) {
        magnetometer_update_calibration(x, y, z);
    }

    return true;
}

bool magnetometer_read_data(magnetometer_data_t *data) {
    if (data == NULL) return false;

    // Read raw values
    int16_t raw_x, raw_y, raw_z;
    if (!magnetometer_read_raw(&raw_x, &raw_y, &raw_z)) {
        return false;
    }

    // Apply moving average filter
    int16_t smooth_x, smooth_y, smooth_z;
    apply_moving_average(raw_x, raw_y, raw_z, &smooth_x, &smooth_y, &smooth_z);

    // Store smoothed values
    data->x = smooth_x;
    data->y = smooth_y;
    data->z = smooth_z;
    data->heading = magnetometer_calculate_heading(smooth_x, smooth_y);
    
    return true;
}

bool magnetometer_read_data_raw(magnetometer_data_t *data) {
    if (data == NULL) return false;

    if (!magnetometer_read_raw(&data->x, &data->y, &data->z)) {
        return false;
    }

    data->heading = magnetometer_calculate_heading(data->x, data->y);
    return true;
}

float magnetometer_calculate_heading(int16_t mx, int16_t my) {
    // Apply calibration (hard-iron correction)
    float x_calibrated = mx - (mag_max_x + mag_min_x) / 2.0f;
    float y_calibrated = my - (mag_max_y + mag_min_y) / 2.0f;

    // Calculate heading in radians
    float heading_rad = atan2f(y_calibrated, x_calibrated);

    // Convert to degrees
    float heading_deg = heading_rad * (180.0f / M_PI);

    // Normalize to 0-360
    if (heading_deg < 0) {
        heading_deg += 360.0f;
    }

    return heading_deg;
}

float magnetometer_get_heading(void) {
    magnetometer_data_t data;
    if (magnetometer_read_data(&data)) {
        return data.heading;
    }
    return NAN;
}

float magnetometer_get_heading_raw(void) {
    int16_t mx, my, mz;
    if (magnetometer_read_raw(&mx, &my, &mz)) {
        return magnetometer_calculate_heading(mx, my);
    }
    return NAN;
}

void magnetometer_print_data(void) {
    magnetometer_data_t data;
    
    if (magnetometer_read_data(&data)) {
        printf("[MAG] X=%6d Y=%6d Z=%6d | Heading=%.1f° ", 
               data.x, data.y, data.z, data.heading);
        
        // Print compass direction
        if (data.heading >= 337.5f || data.heading < 22.5f) {
            printf("(N)\n");
        } else if (data.heading >= 22.5f && data.heading < 67.5f) {
            printf("(NE)\n");
        } else if (data.heading >= 67.5f && data.heading < 112.5f) {
            printf("(E)\n");
        } else if (data.heading >= 112.5f && data.heading < 157.5f) {
            printf("(SE)\n");
        } else if (data.heading >= 157.5f && data.heading < 202.5f) {
            printf("(S)\n");
        } else if (data.heading >= 202.5f && data.heading < 247.5f) {
            printf("(SW)\n");
        } else if (data.heading >= 247.5f && data.heading < 292.5f) {
            printf("(W)\n");
        } else {
            printf("(NW)\n");
        }
    } else {
        printf("[MAG] Read failed\n");
    }
}

void magnetometer_start_calibration(void) {
    mag_min_x = mag_min_y = mag_min_z = 32767;
    mag_max_x = mag_max_y = mag_max_z = -32768;
    calibration_active = true;
    printf("[MAG] Calibration started - rotate sensor in all directions\n");
}

void magnetometer_update_calibration(int16_t mx, int16_t my, int16_t mz) {
    if (!calibration_active) return;

    if (mx < mag_min_x) mag_min_x = mx;
    if (mx > mag_max_x) mag_max_x = mx;
    if (my < mag_min_y) mag_min_y = my;
    if (my > mag_max_y) mag_max_y = my;
    if (mz < mag_min_z) mag_min_z = mz;
    if (mz > mag_max_z) mag_max_z = mz;
}

void magnetometer_finish_calibration(void) {
    calibration_active = false;
    printf("[MAG] Calibration complete:\n");
    printf("  X: [%d, %d]\n", mag_min_x, mag_max_x);
    printf("  Y: [%d, %d]\n", mag_min_y, mag_max_y);
    printf("  Z: [%d, %d]\n", mag_min_z, mag_max_z);
}