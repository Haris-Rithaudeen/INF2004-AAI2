#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdint.h>

// Magnetometer data structure
typedef struct {
    int16_t x;      // Raw X-axis magnetic field
    int16_t y;      // Raw Y-axis magnetic field
    int16_t z;      // Raw Z-axis magnetic field
    float heading;  // Calculated heading in degrees (0-360)
} magnetometer_data_t;

// Initialize the LSM303 magnetometer (and I2C if not already initialized)
bool magnetometer_init(void);

// Read raw magnetometer values (x, y, z) - unfiltered
bool magnetometer_read_raw(int16_t *mx, int16_t *my, int16_t *mz);

// Read magnetometer data structure with smoothed values
bool magnetometer_read_data(magnetometer_data_t *data);

// Read magnetometer data without filtering (raw data)
bool magnetometer_read_data_raw(magnetometer_data_t *data);

// Calculate heading (compass direction) in degrees (0-360)
// 0° = North, 90° = East, 180° = South, 270° = West
float magnetometer_calculate_heading(int16_t mx, int16_t my);

// Get current heading in degrees (with smoothing)
float magnetometer_get_heading(void);

// Get current heading in degrees (without smoothing)
float magnetometer_get_heading_raw(void);

// Print magnetometer data for debugging
void magnetometer_print_data(void);

// Calibration functions (optional, for improved accuracy)
void magnetometer_start_calibration(void);
void magnetometer_update_calibration(int16_t mx, int16_t my, int16_t mz);
void magnetometer_finish_calibration(void);

// Reset the moving average filter
void magnetometer_reset_filter(void);
