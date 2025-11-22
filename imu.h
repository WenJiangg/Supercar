/**
 * ==============================================================================
 * IMU.H - Public Interface for IMU Module (LIS3DH + LSM303)
 * ==============================================================================
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

// ===== Public Data Types =====
typedef struct {
    int16_t accel_x, accel_y, accel_z;    // raw
    int16_t mag_x,   mag_y,   mag_z;      // raw
    float   accel_x_g, accel_y_g, accel_z_g;
    float   mag_x_u,   mag_y_u,   mag_z_u;
    float   heading_deg;
    float   tilt_deg;
} imu_data_t;

// ===== Public API Functions =====

/**
 * @brief Initializes the I2C bus and configures the IMU sensors.
 * @return bool True if both accelerometer and magnetometer are found.
 */
bool imu_init(void);

/**
 * @brief Reads the latest data from the IMU sensors.
 * Call this once per loop.
 */
void imu_read(void);

/**
 * @brief Checks if the IMU was successfully initialized.
 */
bool imu_is_available(void);

/**
 * @brief Returns the detected model string (e.g., "LIS3DH + LSM303D").
 */
const char* imu_get_model(void);

/**
 * @brief Gets a pointer to the internal struct holding the latest IMU data.
 */
const imu_data_t* imu_get_data(void);

/**
 * @brief Helper function to get just the calculated heading.
 */
float imu_get_heading(void);

/**
 * @brief Helper function to get just the calculated tilt.
 */
float imu_get_tilt(void);

#endif // IMU_H