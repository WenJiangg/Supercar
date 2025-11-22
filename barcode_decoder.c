/**
 * ==============================================================================
 * BARCODE_DECODER.C - Implementation of Barcode Decoder
 * ==============================================================================
 * Uses GP28 (ADC2) with analog reading for TCRT5000
 */

#include "barcode_decoder.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// GPIO Pin for Barcode IR Sensor (TCRT5000)
#define BARCODE_IR_PIN 28        // GP28 for barcode reading
#define BARCODE_ADC_CHANNEL 2    // ADC2 for GP28

// Threshold values (will be calibrated)
static uint16_t barcode_threshold = 2000;  // Middle value, will be calibrated
static uint16_t white_value = 500;         // Will be calibrated
static uint16_t black_value = 3500;        // Will be calibrated

// Static variables for scanning
static volatile bool scanning_active = false;
static volatile bool scan_complete = false;
static barcode_t current_barcode;
static absolute_time_t last_transition_time;
static bool last_sensor_state = false;
static uint32_t min_bar_threshold_us = MIN_BAR_WIDTH_US;

// Code39 character set - Full support
// Code39 uses 9 elements per character (5 bars, 4 spaces)
// 3 of the 9 elements are wide, 6 are narrow
// Pattern format: [bar space bar space bar space bar space bar]
// 1=wide, 0=narrow

typedef struct {
    char character;
    uint8_t pattern[9];  // 9 elements: alternating bars and spaces
} code39_char_t;

static const code39_char_t CODE39_TABLE[] = {
    // Numbers
    {'0', {0,0,0,1,1,0,1,0,0}}, // 0
    {'1', {1,0,0,1,0,0,0,0,1}}, // 1
    {'2', {0,0,1,1,0,0,0,0,1}}, // 2
    {'3', {1,0,1,1,0,0,0,0,0}}, // 3
    {'4', {0,0,0,1,1,0,0,0,1}}, // 4
    {'5', {1,0,0,1,1,0,0,0,0}}, // 5
    {'6', {0,0,1,1,1,0,0,0,0}}, // 6
    {'7', {0,0,0,1,0,0,1,0,1}}, // 7
    {'8', {1,0,0,1,0,0,1,0,0}}, // 8
    {'9', {0,0,1,1,0,0,1,0,0}}, // 9
    
    // Letters A-Z
    {'A', {1,0,0,0,0,1,0,0,1}},
    {'B', {0,0,1,0,0,1,0,0,1}},
    {'C', {1,0,1,0,0,1,0,0,0}},
    {'D', {0,0,0,0,1,1,0,0,1}},
    {'E', {1,0,0,0,1,1,0,0,0}},
    {'F', {0,0,1,0,1,1,0,0,0}},
    {'G', {0,0,0,0,0,1,1,0,1}},
    {'H', {1,0,0,0,0,1,1,0,0}},
    {'I', {0,0,1,0,0,1,1,0,0}},
    {'J', {0,0,0,0,1,1,1,0,0}},
    {'K', {1,0,0,0,0,0,0,1,1}},
    {'L', {0,0,1,0,0,0,0,1,1}},
    {'M', {1,0,1,0,0,0,0,1,0}},
    {'N', {0,0,0,0,1,0,0,1,1}},
    {'O', {1,0,0,0,1,0,0,1,0}},
    {'P', {0,0,1,0,1,0,0,1,0}},
    {'Q', {0,0,0,0,0,0,1,1,1}},
    {'R', {1,0,0,0,0,0,1,1,0}},
    {'S', {0,0,1,0,0,0,1,1,0}},
    {'T', {0,0,0,0,1,0,1,1,0}},
    {'U', {1,1,0,0,0,0,0,0,1}},
    {'V', {0,1,1,0,0,0,0,0,1}},
    {'W', {1,1,1,0,0,0,0,0,0}},
    {'X', {0,1,0,0,1,0,0,0,1}},
    {'Y', {1,1,0,0,1,0,0,0,0}},
    {'Z', {0,1,1,0,1,0,0,0,0}},
    
    // Special characters
    {'-', {0,1,0,0,0,0,1,0,1}},
    {'.', {1,1,0,0,0,0,1,0,0}},
    {' ', {0,1,1,0,0,0,1,0,0}},
    {'$', {0,1,0,1,0,1,0,0,0}},
    {'/', {0,1,0,1,0,0,0,1,0}},
    {'+', {0,1,0,0,0,1,0,1,0}},
    {'%', {0,0,0,1,0,1,0,1,0}},
    {'*', {0,1,0,0,1,0,1,0,0}}, // Start/Stop character
};

#define CODE39_TABLE_SIZE (sizeof(CODE39_TABLE) / sizeof(code39_char_t))
#define CODE39_START_STOP '*'

// === Private Function Declarations ===
static void barcode_gpio_callback(uint gpio, uint32_t events);
static bool barcode_classify_bars(bar_t* bars, uint8_t count, uint8_t* pattern);
static uint32_t barcode_get_avg_narrow_width(bar_t* bars, uint8_t count);
static char barcode_decode_code39_char(uint8_t* pattern);
static bool barcode_decode_code39(barcode_t* bc);
static bool barcode_match_pattern(uint8_t* pattern1, uint8_t* pattern2, uint8_t length);

// === Public Function Implementations ===

void barcode_init(void) {
    // Initialize ADC
    adc_init();
    adc_gpio_init(BARCODE_IR_PIN);
    
    printf("✓ Barcode decoder initialized on GP%d (ADC%d)\n", 
           BARCODE_IR_PIN, BARCODE_ADC_CHANNEL);
    printf("  Using ADC mode with threshold: %d\n", barcode_threshold);
}

bool barcode_read_sensor(void) {
    // Read ADC value and compare to threshold
    adc_select_input(BARCODE_ADC_CHANNEL);
    uint16_t value = adc_read();
    return (value > barcode_threshold);  // true = black, false = white
}

/**
 * @brief Read raw ADC value
 */
uint16_t barcode_read_raw(void) {
    adc_select_input(BARCODE_ADC_CHANNEL);
    return adc_read();  // Returns 0-4095
}

/**
 * @brief Read sensor and print raw value continuously for calibration
 * This helps you understand what values your sensor gives
 */
void barcode_calibrate_sensor(void) {
    printf("\n=== TCRT5000 ADC Calibration (GP28) ===\n");
    printf("This will find the correct threshold for your sensor.\n\n");
    
    // Clear any pending input
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    
    printf("Step 1: Place sensor over WHITE paper (5mm distance)\n");
    printf("Press any key when ready...\n");
    while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT);
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    sleep_ms(500);
    
    uint32_t white_sum = 0;
    printf("Sampling white paper");
    fflush(stdout);
    for (int i = 0; i < 100; i++) {
        white_sum += barcode_read_raw();
        if (i % 20 == 0) {
            printf(".");
            fflush(stdout);
        }
        sleep_ms(10);
    }
    white_value = white_sum / 100;
    printf("\n✓ White value: %d\n\n", white_value);
    
    printf("Step 2: Place sensor over BLACK line/paper\n");
    printf("Press any key when ready...\n");
    while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT);
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    sleep_ms(500);
    
    uint32_t black_sum = 0;
    printf("Sampling black surface");
    fflush(stdout);
    for (int i = 0; i < 100; i++) {
        black_sum += barcode_read_raw();
        if (i % 20 == 0) {
            printf(".");
            fflush(stdout);
        }
        sleep_ms(10);
    }
    black_value = black_sum / 100;
    printf("\n✓ Black value: %d\n\n", black_value);
    
    // Calculate threshold (midpoint)
    barcode_threshold = (white_value + black_value) / 2;
    
    printf("=== Calibration Complete! ===\n");
    printf("White value:  %d\n", white_value);
    printf("Black value:  %d\n", black_value);
    printf("Threshold:    %d\n", barcode_threshold);
    printf("Range:        %d\n\n", black_value - white_value);
    
    if (black_value - white_value < 500) {
        printf("⚠️  WARNING: Small range! Sensor may not work well.\n");
        printf("   Try adjusting sensor distance (3-8mm) or check connections.\n\n");
    } else {
        printf("✅ Good range! Sensor should work well.\n\n");
    }
    
    // Clear buffer
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
}

barcode_t barcode_scan(uint32_t timeout_ms) {
    barcode_t result;
    memset(&result, 0, sizeof(barcode_t));
    
    printf("\n=== Starting Barcode Scan ===\n");
    printf("Move barcode under GP%d sensor...\n", BARCODE_IR_PIN);
    
    absolute_time_t start_time = get_absolute_time();
    absolute_time_t timeout_time = delayed_by_ms(start_time, timeout_ms);
    
    bool last_state = barcode_read_sensor();
    absolute_time_t last_time = get_absolute_time();
    
    uint8_t bar_index = 0;
    bool started = false;
    bool in_barcode = false;
    uint32_t stable_white_count = 0;
    
    // Wait for first transition (start of barcode)
    while (bar_index < BARCODE_MAX_BARS) {
        // Check timeout
        if (timeout_ms > 0 && absolute_time_diff_us(get_absolute_time(), timeout_time) >= 0) {
            printf("Barcode scan timeout\n");
            return result;
        }
        
        bool current_state = barcode_read_sensor();
        absolute_time_t current_time = get_absolute_time();
        
        // Detect state change
        if (current_state != last_state) {
            uint32_t width_us = absolute_time_diff_us(last_time, current_time);
            
            // Start detection: first black bar
            if (!started && current_state == true && width_us > MIN_BAR_WIDTH_US) {
                started = true;
                in_barcode = true;
                printf("Barcode detected! Reading...\n");
            }
            
            // Record bars and spaces
            if (started && width_us >= MIN_BAR_WIDTH_US && width_us <= MAX_BAR_WIDTH_US) {
                result.bars[bar_index].width_us = width_us;
                result.bars[bar_index].is_black = last_state;
                bar_index++;
                
                if (bar_index % 10 == 0) {
                    printf(".");
                    fflush(stdout);
                }
                
                stable_white_count = 0;
            } else if (started && !current_state) {
                // Count consecutive white spaces
                stable_white_count++;
                if (stable_white_count > 5) {
                    // End of barcode detected
                    printf("\nBarcode end detected\n");
                    break;
                }
            }
            
            last_state = current_state;
            last_time = current_time;
        }
        
        sleep_us(50);  // Small delay to avoid too fast polling
    }
    
    result.bar_count = bar_index;
    printf("\nCaptured %d bars/spaces\n", result.bar_count);
    
    // Decode the barcode
    if (result.bar_count > 0) {
        result.type = barcode_decode_bars(result.bars, result.bar_count, 
                                         result.data, BARCODE_MAX_DATA_LEN);
        result.valid = (result.type != BARCODE_UNKNOWN && result.data_length > 0);
        
        if (result.valid) {
            printf("✓ Barcode decoded: %s\n", result.data);
        } else {
            printf("✗ Unable to decode barcode\n");
        }
    }
    
    return result;
}

barcode_type_t barcode_decode_bars(bar_t* bars, uint8_t bar_count, char* output, uint8_t max_len) {
    if (bar_count < 20) {  // Code39 needs at least 20 bars for minimal barcode
        printf("Too few bars: %d (need at least 20)\n", bar_count);
        return BARCODE_UNKNOWN;
    }
    
    // Calculate average narrow bar width for classification
    uint32_t avg_narrow = barcode_get_avg_narrow_width(bars, bar_count);
    uint32_t threshold = avg_narrow * 1.5;  // 1.5x narrow = wide threshold
    
    printf("Average narrow width: %u us, threshold: %u us\n", avg_narrow, threshold);
    
    // Try Code39 decoding first
    barcode_t temp_bc;
    memcpy(&temp_bc, bars, sizeof(bar_t) * bar_count);
    temp_bc.bar_count = bar_count;
    
    if (barcode_decode_code39(&temp_bc)) {
        strcpy(output, temp_bc.data);
        printf("✓ Decoded as Code39: %s\n", output);
        return BARCODE_CODE39;
    }
    
    // Fallback: Simple binary decoding
    printf("Code39 failed, trying simple binary...\n");
    uint8_t byte_value = 0;
    uint8_t bit_count = 0;
    uint8_t output_index = 0;
    
    for (uint8_t i = 0; i < bar_count && output_index < max_len; i++) {
        if (bars[i].is_black) {  // Only decode black bars
            bool is_wide = (bars[i].width_us > threshold);
            
            byte_value = (byte_value << 1) | (is_wide ? 1 : 0);
            bit_count++;
            
            // Every 8 bits = 1 character
            if (bit_count == 8) {
                if (byte_value >= 32 && byte_value <= 126) {  // Printable ASCII
                    output[output_index++] = (char)byte_value;
                }
                byte_value = 0;
                bit_count = 0;
            }
        }
    }
    
    output[output_index] = '\0';
    
    // If we got some valid data, return custom type
    if (output_index > 0) {
        printf("✓ Decoded as Custom: %s\n", output);
        return BARCODE_CUSTOM;
    }
    
    printf("✗ Unable to decode barcode\n");
    return BARCODE_UNKNOWN;
}

bool barcode_scan_start(void) {
    if (scanning_active) {
        return false;
    }
    
    memset(&current_barcode, 0, sizeof(barcode_t));
    scanning_active = true;
    scan_complete = false;
    last_sensor_state = barcode_read_sensor();
    last_transition_time = get_absolute_time();
    
    printf("Barcode scan started (non-blocking)\n");
    return true;
}

barcode_t* barcode_scan_check(void) {
    if (!scanning_active) {
        return NULL;
    }
    
    if (scan_complete) {
        scanning_active = false;
        return &current_barcode;
    }
    
    // Poll sensor and update
    bool current_state = barcode_read_sensor();
    
    if (current_state != last_sensor_state) {
        absolute_time_t current_time = get_absolute_time();
        uint32_t width_us = absolute_time_diff_us(last_transition_time, current_time);
        
        if (width_us >= MIN_BAR_WIDTH_US && width_us <= MAX_BAR_WIDTH_US) {
            if (current_barcode.bar_count < BARCODE_MAX_BARS) {
                current_barcode.bars[current_barcode.bar_count].width_us = width_us;
                current_barcode.bars[current_barcode.bar_count].is_black = last_sensor_state;
                current_barcode.bar_count++;
            }
        }
        
        last_sensor_state = current_state;
        last_transition_time = current_time;
    }
    
    // Check for end condition (long white space)
    if (!current_state) {
        uint32_t white_duration = absolute_time_diff_us(last_transition_time, get_absolute_time());
        if (white_duration > 10000 && current_barcode.bar_count > 0) {  // 10ms white = end
            scan_complete = true;
            
            // Decode
            current_barcode.type = barcode_decode_bars(current_barcode.bars, 
                                                       current_barcode.bar_count,
                                                       current_barcode.data, 
                                                       BARCODE_MAX_DATA_LEN);
            current_barcode.valid = (current_barcode.type != BARCODE_UNKNOWN);
            current_barcode.data_length = strlen(current_barcode.data);
        }
    }
    
    return NULL;
}

void barcode_scan_cancel(void) {
    scanning_active = false;
    scan_complete = false;
    printf("Barcode scan cancelled\n");
}

void barcode_print_info(barcode_t* bc) {
    if (!bc) {
        printf("No barcode data\n");
        return;
    }
    
    printf("\n=== Barcode Information ===\n");
    printf("Type: %s\n", barcode_type_string(bc->type));
    printf("Valid: %s\n", bc->valid ? "Yes" : "No");
    printf("Data: \"%s\"\n", bc->data);
    printf("Data Length: %d\n", bc->data_length);
    printf("Bar Count: %d\n", bc->bar_count);
    
    printf("\nBar Pattern (first 20):\n");
    for (uint8_t i = 0; i < bc->bar_count && i < 20; i++) {
        printf("  [%2d] %s: %4u us\n", i, 
               bc->bars[i].is_black ? "BLACK" : "WHITE",
               bc->bars[i].width_us);
    }
    printf("\n");
}

const char* barcode_type_string(barcode_type_t type) {
    switch (type) {
        case BARCODE_CODE39:  return "Code39";
        case BARCODE_CODE128: return "Code128";
        case BARCODE_CUSTOM:  return "Custom";
        default:              return "Unknown";
    }
}

void barcode_set_threshold(uint32_t threshold_us) {
    min_bar_threshold_us = threshold_us;
    printf("Barcode threshold set to %u us\n", threshold_us);
}

bool barcode_decode_simple_binary(barcode_t* bc) {
    if (!bc || bc->bar_count < 8) {
        return false;
    }
    
    uint32_t avg_narrow = barcode_get_avg_narrow_width(bc->bars, bc->bar_count);
    
    // Simple 4-bit encoding: each digit 0-9 uses 4 bars
    // Pattern: NNNN=0, NNNW=1, NNWN=2, etc.
    uint8_t output_idx = 0;
    
    for (uint8_t i = 0; i < bc->bar_count - 3 && output_idx < BARCODE_MAX_DATA_LEN; i += 4) {
        // Check if all 4 are black bars
        if (!bc->bars[i].is_black || !bc->bars[i+1].is_black || 
            !bc->bars[i+2].is_black || !bc->bars[i+3].is_black) {
            continue;
        }
        
        // Classify each bar as narrow (0) or wide (1)
        uint8_t pattern = 0;
        for (uint8_t j = 0; j < 4; j++) {
            bool is_wide = (bc->bars[i+j].width_us > (avg_narrow * 1.5));
            pattern = (pattern << 1) | (is_wide ? 1 : 0);
        }
        
        // Convert pattern to digit
        if (pattern < 10) {
            bc->data[output_idx++] = '0' + pattern;
        }
    }
    
    bc->data[output_idx] = '\0';
    bc->data_length = output_idx;
    
    return (output_idx > 0);
}

// === Private Functions ===

static uint32_t barcode_get_avg_narrow_width(bar_t* bars, uint8_t count) {
    uint32_t sum = 0;
    uint8_t narrow_count = 0;
    
    // Find all bars that are likely "narrow"
    for (uint8_t i = 0; i < count; i++) {
        if (bars[i].width_us < NARROW_BAR_MAX_US) {
            sum += bars[i].width_us;
            narrow_count++;
        }
    }
    
    if (narrow_count > 0) {
        return sum / narrow_count;
    }
    
    return 150;  // Default narrow width
}

static bool barcode_classify_bars(bar_t* bars, uint8_t count, uint8_t* pattern) {
    uint32_t avg_narrow = barcode_get_avg_narrow_width(bars, count);
    
    for (uint8_t i = 0; i < count && i < 9; i++) {
        pattern[i] = (bars[i].width_us > (avg_narrow * 1.5)) ? 1 : 0;
    }
    
    return true;
}

static bool barcode_match_pattern(uint8_t* pattern1, uint8_t* pattern2, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        if (pattern1[i] != pattern2[i]) {
            return false;
        }
    }
    return true;
}

static char barcode_decode_code39_char(uint8_t* pattern) {
    // Match pattern against Code39 table
    for (uint8_t i = 0; i < CODE39_TABLE_SIZE; i++) {
        if (barcode_match_pattern(pattern, (uint8_t*)CODE39_TABLE[i].pattern, 9)) {
            return CODE39_TABLE[i].character;
        }
    }
    return '\0';  // No match
}

static bool barcode_decode_code39(barcode_t* bc) {
    if (bc->bar_count < 20) {
        return false;
    }
    
    // Calculate threshold for narrow/wide classification
    uint32_t avg_narrow = barcode_get_avg_narrow_width(bc->bars, bc->bar_count);
    uint32_t threshold = avg_narrow * 1.5;
    
    printf("\n=== Code39 Decoding ===\n");
    printf("Avg narrow: %u us, Threshold: %u us\n", avg_narrow, threshold);
    
    // Convert bars to pattern (0=narrow, 1=wide)
    uint8_t* bar_pattern = (uint8_t*)malloc(bc->bar_count);
    if (!bar_pattern) {
        return false;
    }
    
    for (uint8_t i = 0; i < bc->bar_count; i++) {
        bar_pattern[i] = (bc->bars[i].width_us > threshold) ? 1 : 0;
    }
    
    // Print bar pattern for debugging
    printf("Bar pattern: ");
    for (uint8_t i = 0; i < bc->bar_count && i < 50; i++) {
        printf("%d", bar_pattern[i]);
        if ((i + 1) % 9 == 0) printf(" ");
    }
    printf("\n");
    
    // Find start character (*)
    bool found_start = false;
    uint8_t start_index = 0;
    uint8_t pattern_buffer[9];
    
    for (uint8_t i = 0; i <= bc->bar_count - 9; i++) {
        // Extract 9 elements (1 character in Code39)
        for (uint8_t j = 0; j < 9; j++) {
            pattern_buffer[j] = bar_pattern[i + j];
        }
        
        char decoded = barcode_decode_code39_char(pattern_buffer);
        if (decoded == CODE39_START_STOP) {
            found_start = true;
            start_index = i + 9;  // Skip start character
            printf("Found start character at index %d\n", i);
            break;
        }
    }
    
    if (!found_start) {
        printf("✗ No start character found\n");
        free(bar_pattern);
        return false;
    }
    
    // Decode characters until stop character or end
    uint8_t data_index = 0;
    uint8_t char_index = start_index;
    
    printf("Decoding characters:\n");
    
    while (char_index + 9 <= bc->bar_count && data_index < BARCODE_MAX_DATA_LEN) {
        // Extract 9 elements
        for (uint8_t j = 0; j < 9; j++) {
            pattern_buffer[j] = bar_pattern[char_index + j];
        }
        
        char decoded = barcode_decode_code39_char(pattern_buffer);
        
        printf("  [%3d] Pattern: ", char_index);
        for (uint8_t j = 0; j < 9; j++) printf("%d", pattern_buffer[j]);
        printf(" → '%c' (%d)\n", decoded ? decoded : '?', decoded);
        
        if (decoded == CODE39_START_STOP) {
            // Found stop character - end of barcode
            printf("Found stop character at index %d\n", char_index);
            break;
        } else if (decoded != '\0') {
            // Valid character
            bc->data[data_index++] = decoded;
        } else {
            // Invalid character - might be noise or inter-character gap
            printf("  Warning: Invalid pattern at index %d\n", char_index);
        }
        
        char_index += 9;  // Move to next character
    }
    
    bc->data[data_index] = '\0';
    bc->data_length = data_index;
    
    free(bar_pattern);
    
    if (data_index > 0) {
        printf("✓ Code39 decoded: \"%s\" (%d chars)\n", bc->data, data_index);
        return true;
    }
    
    printf("✗ No valid data decoded\n");
    return false;
}