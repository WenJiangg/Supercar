/**
 * ==============================================================================
 * BARCODE_DECODER.H - Barcode Decoder for Right IR Sensor (GPIO3)
 * ==============================================================================
 * This module handles barcode reading using the right IR sensor on GPIO3.
 * Supports basic barcode formats for instruction decoding.
 */

#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

// Barcode configuration
#define BARCODE_MAX_BARS 50       // Maximum number of bars to detect
#define BARCODE_MAX_DATA_LEN 20   // Maximum decoded data length
#define BARCODE_TIMEOUT_MS 5000   // Timeout for barcode scanning

// Bar width thresholds (in microseconds)
#define MIN_BAR_WIDTH_US 50       // Minimum valid bar width
#define MAX_BAR_WIDTH_US 5000     // Maximum valid bar width
#define NARROW_BAR_MAX_US 300     // Narrow bar threshold
#define WIDE_BAR_MIN_US 400       // Wide bar threshold

// Barcode types
typedef enum {
    BARCODE_UNKNOWN = 0,
    BARCODE_CODE39,
    BARCODE_CODE128,
    BARCODE_CUSTOM
} barcode_type_t;

// Bar structure
typedef struct {
    uint32_t width_us;    // Width in microseconds
    bool is_black;        // true = black bar, false = white space
} bar_t;

// Barcode data structure
typedef struct {
    barcode_type_t type;
    char data[BARCODE_MAX_DATA_LEN + 1];  // Decoded data (null-terminated)
    uint8_t data_length;
    bar_t bars[BARCODE_MAX_BARS];
    uint8_t bar_count;
    bool valid;
} barcode_t;

/**
 * @brief Initialize the barcode decoder on GPIO3
 * Sets up the GPIO pin and interrupt handling
 */
void barcode_init(void);

/**
 * @brief Start scanning for a barcode
 * This is a blocking function that waits for a barcode to pass under the sensor
 * @param timeout_ms Timeout in milliseconds (0 = no timeout)
 * @return barcode_t structure with decoded data
 */
barcode_t barcode_scan(uint32_t timeout_ms);

/**
 * @brief Start scanning for a barcode (non-blocking)
 * Call barcode_scan_check() to check if scanning is complete
 * @return true if scan started successfully
 */
bool barcode_scan_start(void);

/**
 * @brief Check if barcode scanning is complete
 * @return Pointer to barcode data if complete, NULL if still scanning
 */
barcode_t* barcode_scan_check(void);

/**
 * @brief Cancel ongoing barcode scan
 */
void barcode_scan_cancel(void);

/**
 * @brief Read raw sensor value from GPIO28 (ADC)
 * @return true if black detected (bar), false if white (space)
 */
bool barcode_read_sensor(void);

/**
 * @brief Read raw ADC value (0-4095)
 * @return Raw ADC reading
 */
uint16_t barcode_read_raw(void);

/**
 * @brief Calibrate the IR sensor to find threshold
 * Interactive calibration for white/black values
 */
void barcode_calibrate_sensor(void);

/**
 * @brief Decode raw bar data into a barcode string
 * @param bars Array of bar structures
 * @param bar_count Number of bars detected
 * @param output Output buffer for decoded string
 * @param max_len Maximum output buffer length
 * @return barcode_type_t detected type
 */
barcode_type_t barcode_decode_bars(bar_t* bars, uint8_t bar_count, char* output, uint8_t max_len);

/**
 * @brief Print barcode information for debugging
 * @param bc Pointer to barcode structure
 */
void barcode_print_info(barcode_t* bc);

/**
 * @brief Convert barcode type to string
 * @param type Barcode type
 * @return String representation
 */
const char* barcode_type_string(barcode_type_t type);

/**
 * @brief Set calibration threshold for black/white detection
 * @param threshold_us Minimum width to consider as valid transition
 */
void barcode_set_threshold(uint32_t threshold_us);

/**
 * @brief Decode simple binary barcode pattern (custom format)
 * Uses narrow/wide bar encoding
 * @param bc Pointer to barcode structure
 * @return true if decoded successfully
 */
bool barcode_decode_simple_binary(barcode_t* bc);

#endif // BARCODE_DECODER_H