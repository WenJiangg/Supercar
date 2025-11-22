/**
 * ==============================================================================
 * IR_SENSOR.H - Public Interface for IR Sensor Module
 * ==============================================================================
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

// Line detection structure
typedef struct {
    uint16_t raw_value;
    float normalized_value;  // 0.0 (white) to 1.0 (black)
    bool is_on_line;
} line_detection_t;

/**
 * @brief Initializes the ADC for the line sensor.
 */
void ir_sensor_init(void);

/**
 * @brief Reads the line sensor and returns a full data structure.
 * @return line_detection_t struct with raw, normalized, and is_on_line data.
 */
line_detection_t ir_read_line(void);

/**
 * @brief Helper function for testing, converts normalized to -1.0 to +1.0
 */
float ir_get_line_position(void);

// <-- ir_calibrate() function declaration removed -->

#endif // IR_SENSOR_H