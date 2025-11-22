/**
 * ==============================================================================
 * BARCODE_DIRECTION.H - CODE39 Barcode Direction Detection System
 * ==============================================================================
 * This module provides enhanced CODE39 barcode decoding with:
 * - Wide/Narrow bar classification
 * - Forward/Reverse direction detection
 * - Automatic direction-based motor control commands
 *
 * Hardware: Single IR sensor on GPIO28 (ADC2)
 * ==============================================================================
 */

#ifndef BARCODE_DIRECTION_H
#define BARCODE_DIRECTION_H

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

// ==============================================================================
// Configuration Constants
// ==============================================================================

#define BC_DIR_GPIO_PIN         28          // GPIO28 for barcode IR sensor
#define BC_DIR_ADC_CHANNEL      2           // ADC2 for GPIO28

// Timing thresholds (microseconds)
#define BC_DIR_MIN_BAR_WIDTH    50          // Minimum valid bar width
#define BC_DIR_MAX_BAR_WIDTH    10000       // Maximum valid bar width
#define BC_DIR_SCAN_TIMEOUT_MS  5000        // Scan timeout

// Buffer sizes
#define BC_DIR_MAX_BARS         100         // Maximum bars to capture
#define BC_DIR_MAX_DATA_LEN     30          // Maximum decoded characters

// ==============================================================================
// Enumerations
// ==============================================================================

/**
 * @brief Movement direction based on decoded character
 */
typedef enum {
    MOVE_NONE = 0,      // No movement or invalid character
    MOVE_RIGHT,         // Characters: A, C, E, G, I, K, M, O, Q, S, U, W, Y
    MOVE_LEFT           // Characters: B, D, F, H, J, L, N, P, R, T, V, X, Z
} movement_direction_t;

/**
 * @brief Barcode scan direction (auto-detected)
 */
typedef enum {
    SCAN_UNKNOWN = 0,
    SCAN_FORWARD,       // Left to right (normal)
    SCAN_REVERSE        // Right to left (reversed)
} scan_direction_t;

/**
 * @brief Barcode decode status
 */
typedef enum {
    BC_STATUS_OK = 0,           // Successfully decoded
    BC_STATUS_TIMEOUT,          // Scan timeout
    BC_STATUS_NO_START,         // No start character found
    BC_STATUS_NO_STOP,          // No stop character found
    BC_STATUS_INVALID_PATTERN,  // Invalid bar pattern
    BC_STATUS_TOO_FEW_BARS,     // Not enough bars captured
    BC_STATUS_CHECKSUM_ERROR    // Checksum validation failed (if enabled)
} barcode_status_t;

// ==============================================================================
// Data Structures
// ==============================================================================

/**
 * @brief Individual bar/space element
 */
typedef struct {
    uint32_t width_us;      // Width in microseconds
    bool is_black;          // true = black bar, false = white space
    bool is_wide;           // true = wide, false = narrow (after classification)
} bar_element_t;

/**
 * @brief Complete barcode scan result
 */
typedef struct {
    char data[BC_DIR_MAX_DATA_LEN + 1];     // Decoded string (null-terminated)
    uint8_t data_length;                     // Number of decoded characters

    scan_direction_t scan_dir;               // Detected scan direction
    movement_direction_t move_dir;           // Recommended movement direction

    barcode_status_t status;                 // Decode status

    bar_element_t bars[BC_DIR_MAX_BARS];    // Raw bar data
    uint16_t bar_count;                      // Number of bars captured

    uint32_t avg_narrow_width_us;           // Calculated average narrow width
    uint32_t threshold_us;                   // Wide/narrow threshold used

    uint32_t scan_duration_ms;              // Total scan time
} barcode_result_t;

// ==============================================================================
// Public API Functions
// ==============================================================================

/**
 * @brief Initialize the barcode direction detection system
 *
 * Sets up GPIO28 as ADC input and calibrates the sensor threshold
 */
void barcode_direction_init(void);

/**
 * @brief Perform interactive sensor calibration
 *
 * Guides user through white/black calibration to set optimal threshold
 */
void barcode_direction_calibrate(void);

/**
 * @brief Scan and decode a barcode (blocking)
 *
 * This function will wait for a barcode to be scanned and automatically
 * detect direction and decode the content.
 *
 * @param timeout_ms Maximum time to wait for barcode (0 = no timeout)
 * @return barcode_result_t Complete scan result with decoded data
 */
barcode_result_t barcode_direction_scan(uint32_t timeout_ms);

/**
 * @brief Get movement direction for a specific character
 *
 * @param c Character to check
 * @return movement_direction_t MOVE_RIGHT, MOVE_LEFT, or MOVE_NONE
 */
movement_direction_t barcode_get_move_direction(char c);

/**
 * @brief Get human-readable string for movement direction
 *
 * @param dir Movement direction enum
 * @return const char* String representation
 */
const char* barcode_direction_string(movement_direction_t dir);

/**
 * @brief Get human-readable string for scan direction
 *
 * @param dir Scan direction enum
 * @return const char* String representation
 */
const char* barcode_scan_direction_string(scan_direction_t dir);

/**
 * @brief Get human-readable string for barcode status
 *
 * @param status Barcode status enum
 * @return const char* String representation
 */
const char* barcode_status_string(barcode_status_t status);

/**
 * @brief Print detailed barcode result information
 *
 * @param result Pointer to barcode result structure
 */
void barcode_print_result(const barcode_result_t* result);

/**
 * @brief Execute movement command based on barcode result
 *
 * This is a convenience function that can be integrated with your
 * robot controller to execute the appropriate movement.
 *
 * @param result Pointer to barcode result
 * @return true if movement command was executed
 */
bool barcode_execute_movement(const barcode_result_t* result);

/**
 * @brief Read raw ADC value from sensor
 *
 * @return uint16_t Raw ADC value (0-4095)
 */
uint16_t barcode_direction_read_raw(void);

/**
 * @brief Read sensor state (black/white)
 *
 * @return bool true = black bar, false = white space
 */
bool barcode_direction_read_sensor(void);

/**
 * @brief Set ADC threshold for black/white detection
 *
 * @param threshold ADC threshold value (0-4095)
 */
void barcode_direction_set_threshold(uint16_t threshold);

/**
 * @brief Get current ADC threshold
 *
 * @return uint16_t Current threshold value
 */
uint16_t barcode_direction_get_threshold(void);

// ==============================================================================
// Non-Blocking Continuous Scanning API
// ==============================================================================

/**
 * @brief Start continuous non-blocking barcode scanning
 *
 * This enables background barcode scanning that runs without blocking
 * your robot's main control loop. Call barcode_direction_update() regularly
 * to process scanning.
 */
void barcode_direction_start_continuous(void);

/**
 * @brief Stop continuous scanning
 */
void barcode_direction_stop_continuous(void);

/**
 * @brief Update the barcode scanner state machine (call regularly in main loop)
 *
 * This function should be called frequently (e.g., every 1-5ms) from your
 * main control loop. It processes the scanner state machine and detects
 * barcodes without blocking.
 *
 * @return barcode_result_t* Pointer to result if barcode was just decoded,
 *                           NULL if still scanning or idle
 */
barcode_result_t* barcode_direction_update(void);

/**
 * @brief Check if continuous scanning is active
 *
 * @return bool true if scanning is active
 */
bool barcode_direction_is_scanning(void);

/**
 * @brief Reset the continuous scanner (clears current scan data)
 */
void barcode_direction_reset(void);

#endif // BARCODE_DIRECTION_H
