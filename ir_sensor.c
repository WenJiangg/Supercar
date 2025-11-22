/**
 * ==============================================================================
 * IR_SENSOR.C - Implementation for IR Sensor Module
 * ==============================================================================
 */

#include "ir_sensor.h" // <-- Includes the header file
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// IR Sensor Pin Definition
// Left IR Sensor - Line Detection (Analog via GROVE 6)
#define IR_LEFT_LINE_PIN 27        // GP27 (ADC1 on GROVE 6) - Left IR for line detection

// Calibration values (TUNED based on your actual sensor readings)
#define LINE_THRESHOLD 1900      // Safe threshold between white (220) and black (3600)
#define BLACK_LINE_VALUE 3600    // Your measured black line value
#define WHITE_SURFACE_VALUE 220  // Your measured white surface value

// Calibration values (will now ALWAYS be the default values above)
static uint16_t calibrated_black = BLACK_LINE_VALUE;
static uint16_t calibrated_white = WHITE_SURFACE_VALUE;

// Initialize IR sensor system
void ir_sensor_init() {
    // Initialize ADC for LEFT sensor (line detection on GROVE 6)
    adc_init();
    adc_gpio_init(IR_LEFT_LINE_PIN);
    adc_select_input(1);  // ADC1 for GP27
    
    printf("IR Sensor initialized\n");
    printf("LEFT IR (Line Detection): GP%d (ADC1 on GROVE 6) - Analog\n", IR_LEFT_LINE_PIN);
}

// Read line detection sensor (LEFT sensor - analog)
line_detection_t ir_read_line() {
    line_detection_t result;
    
    // Read ADC value from LEFT sensor (12-bit: 0-4095)
    result.raw_value = adc_read();
    
    // Normalize to 0.0-1.0 range based on calibration
    // Your sensor: WHITE=220, BLACK=3600, Range=3380
    float range = (float)(calibrated_black - calibrated_white);
    if (range > 100) {  // Valid range check
        result.normalized_value = (float)(result.raw_value - calibrated_white) / range;
        // Clamp to 0.0-1.0
        if (result.normalized_value < 0.0f) result.normalized_value = 0.0f;
        if (result.normalized_value > 1.0f) result.normalized_value = 1.0f;
    } else {
        result.normalized_value = 0.5f;
    }
    
    // Determine if on line (higher value = black line)
    result.is_on_line = (result.raw_value > LINE_THRESHOLD);
    
    return result;
}

// Calculate line position (-1.0 to +1.0, 0.0 = centered)
float ir_get_line_position() {
    line_detection_t detection = ir_read_line();
    
    // Convert normalized value to position
    // 0.0 (white) -> -1.0 (far left)
    // 0.5 (gray) -> 0.0 (centered)
    // 1.0 (black) -> +1.0 (far right)
    return (detection.normalized_value - 0.5f) * 2.0f;
}

// <-- ir_calibrate() function body REMOVED -->