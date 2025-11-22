/**
 * ==============================================================================
 * BARCODE_DIRECTION.C - Implementation of CODE39 Barcode Direction Detection
 * ==============================================================================
 */

#include "barcode_direction.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ==============================================================================
// CODE39 Character Encoding Table
// ==============================================================================

/**
 * CODE39 uses 9 elements per character (5 bars + 4 spaces)
 * 3 elements are wide, 6 are narrow
 * Pattern format: [bar space bar space bar space bar space bar]
 * 1 = wide, 0 = narrow
 */
typedef struct {
    char character;
    uint8_t pattern[9];
} code39_char_t;

static const code39_char_t CODE39_TABLE[] = {
    // Numbers
    {'0', {0,0,0,1,1,0,1,0,0}},
    {'1', {1,0,0,1,0,0,0,0,1}},
    {'2', {0,0,1,1,0,0,0,0,1}},
    {'3', {1,0,1,1,0,0,0,0,0}},
    {'4', {0,0,0,1,1,0,0,0,1}},
    {'5', {1,0,0,1,1,0,0,0,0}},
    {'6', {0,0,1,1,1,0,0,0,0}},
    {'7', {0,0,0,1,0,0,1,0,1}},
    {'8', {1,0,0,1,0,0,1,0,0}},
    {'9', {0,0,1,1,0,0,1,0,0}},

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

// ==============================================================================
// Movement Direction Mapping
// ==============================================================================

/**
 * Movement direction rules:
 * RIGHT: A, C, E, G, I, K, M, O, Q, S, U, W, Y
 * LEFT:  B, D, F, H, J, L, N, P, R, T, V, X, Z
 */
static const char RIGHT_CHARS[] = "ACEGIKMOQSUWY";
static const char LEFT_CHARS[] = "BDFHJLNPRTVXZ";

// ==============================================================================
// Static Variables
// ==============================================================================

static uint16_t adc_threshold = 2000;       // ADC threshold for black/white
static uint16_t white_calibration = 500;    // Calibrated white value
static uint16_t black_calibration = 3500;   // Calibrated black value

// ==============================================================================
// Non-Blocking Continuous Scanning State Machine
// ==============================================================================

typedef enum {
    SCAN_STATE_IDLE = 0,        // Not scanning
    SCAN_STATE_WAITING,         // Waiting for first bar
    SCAN_STATE_CAPTURING,       // Capturing bars
    SCAN_STATE_PROCESSING,      // Decoding captured data
    SCAN_STATE_COMPLETE         // Scan complete, result ready
} scan_state_t;

static scan_state_t continuous_scan_state = SCAN_STATE_IDLE;
static barcode_result_t continuous_result;
static bool last_sensor_state_continuous = false;
static absolute_time_t last_transition_time;
static uint32_t stable_white_time_us = 0;
static bool continuous_scanning_enabled = false;

#define BARCODE_END_TIMEOUT_US  15000  // 15ms of white = end of barcode
#define BARCODE_START_MIN_US    100    // Minimum width to start barcode

// ==============================================================================
// Private Function Declarations
// ==============================================================================

static uint32_t calculate_avg_narrow_width(bar_element_t* bars, uint16_t count);
static void classify_bars_wide_narrow(bar_element_t* bars, uint16_t count, uint32_t threshold);
static bool decode_code39_forward(bar_element_t* bars, uint16_t count, char* output, uint8_t max_len);
static bool decode_code39_reverse(bar_element_t* bars, uint16_t count, char* output, uint8_t max_len);
static char decode_code39_character(uint8_t* pattern);
static bool match_pattern(uint8_t* p1, uint8_t* p2, uint8_t len);
static void reverse_string(char* str);
static movement_direction_t determine_movement_direction(const char* data);

// ==============================================================================
// Public API Implementation
// ==============================================================================

void barcode_direction_init(void) {
    // Initialize ADC
    adc_init();
    adc_gpio_init(BC_DIR_GPIO_PIN);

    printf("\n=== CODE39 Barcode Direction Detection System ===\n");
    printf("Hardware: GPIO%d (ADC%d)\n", BC_DIR_GPIO_PIN, BC_DIR_ADC_CHANNEL);
    printf("ADC Threshold: %d\n", adc_threshold);
    printf("\nMovement Rules:\n");
    printf("  RIGHT: %s\n", RIGHT_CHARS);
    printf("  LEFT:  %s\n", LEFT_CHARS);
    printf("=================================================\n\n");
}

void barcode_direction_calibrate(void) {
    printf("\n=== Barcode Sensor Calibration ===\n");
    printf("This will calibrate the IR sensor for optimal barcode detection.\n\n");

    // Clear input buffer
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);

    // Step 1: Calibrate WHITE
    printf("Step 1: Place sensor over WHITE paper\n");
    printf("Press any key when ready...\n");
    while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT);
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    sleep_ms(500);

    uint32_t white_sum = 0;
    printf("Sampling white surface");
    fflush(stdout);
    for (int i = 0; i < 100; i++) {
        white_sum += barcode_direction_read_raw();
        if (i % 20 == 0) {
            printf(".");
            fflush(stdout);
        }
        sleep_ms(10);
    }
    white_calibration = white_sum / 100;
    printf("\nWhite value: %d\n\n", white_calibration);

    // Step 2: Calibrate BLACK
    printf("Step 2: Place sensor over BLACK barcode bar\n");
    printf("Press any key when ready...\n");
    while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT);
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    sleep_ms(500);

    uint32_t black_sum = 0;
    printf("Sampling black surface");
    fflush(stdout);
    for (int i = 0; i < 100; i++) {
        black_sum += barcode_direction_read_raw();
        if (i % 20 == 0) {
            printf(".");
            fflush(stdout);
        }
        sleep_ms(10);
    }
    black_calibration = black_sum / 100;
    printf("\nBlack value: %d\n\n", black_calibration);

    // Calculate threshold
    adc_threshold = (white_calibration + black_calibration) / 2;

    printf("=== Calibration Complete ===\n");
    printf("White:     %d\n", white_calibration);
    printf("Black:     %d\n", black_calibration);
    printf("Threshold: %d\n", adc_threshold);
    printf("Range:     %d\n\n", black_calibration - white_calibration);

    if (black_calibration - white_calibration < 500) {
        printf("WARNING: Small range detected!\n");
        printf("Check sensor distance (optimal: 3-8mm) and connections.\n\n");
    } else {
        printf("Good range! Sensor ready for barcode scanning.\n\n");
    }

    // Clear buffer
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
}

barcode_result_t barcode_direction_scan(uint32_t timeout_ms) {
    barcode_result_t result;
    memset(&result, 0, sizeof(barcode_result_t));
    result.status = BC_STATUS_TIMEOUT;
    result.scan_dir = SCAN_UNKNOWN;
    result.move_dir = MOVE_NONE;

    printf("\n=== Starting CODE39 Barcode Scan ===\n");
    printf("Move barcode under sensor (GPIO%d)...\n", BC_DIR_GPIO_PIN);

    absolute_time_t start_time = get_absolute_time();
    absolute_time_t timeout_time = delayed_by_ms(start_time, timeout_ms);

    bool last_state = barcode_direction_read_sensor();
    absolute_time_t last_transition = get_absolute_time();

    uint16_t bar_index = 0;
    bool scan_started = false;
    uint32_t stable_white_count = 0;

    // Capture bars and spaces
    while (bar_index < BC_DIR_MAX_BARS) {
        // Check timeout
        if (timeout_ms > 0 && absolute_time_diff_us(get_absolute_time(), timeout_time) >= 0) {
            printf("Scan timeout after %d ms\n", timeout_ms);
            return result;
        }

        bool current_state = barcode_direction_read_sensor();
        absolute_time_t current_time = get_absolute_time();

        // Detect state transition
        if (current_state != last_state) {
            uint32_t width_us = absolute_time_diff_us(last_transition, current_time);

            // Start detection on first significant black bar
            if (!scan_started && current_state == true && width_us > BC_DIR_MIN_BAR_WIDTH) {
                scan_started = true;
                printf("Barcode detected! Capturing bars...\n");
            }

            // Record valid bars/spaces
            if (scan_started && width_us >= BC_DIR_MIN_BAR_WIDTH && width_us <= BC_DIR_MAX_BAR_WIDTH) {
                result.bars[bar_index].width_us = width_us;
                result.bars[bar_index].is_black = last_state;
                result.bars[bar_index].is_wide = false; // Will be classified later
                bar_index++;

                if (bar_index % 10 == 0) {
                    printf(".");
                    fflush(stdout);
                }

                stable_white_count = 0;
            } else if (scan_started && !current_state) {
                // Count consecutive white spaces for end detection
                stable_white_count++;
                if (stable_white_count > 3) {
                    printf("\nEnd of barcode detected\n");
                    break;
                }
            }

            last_state = current_state;
            last_transition = current_time;
        }

        sleep_us(50); // Small delay to avoid excessive polling
    }

    result.bar_count = bar_index;
    result.scan_duration_ms = absolute_time_diff_us(start_time, get_absolute_time()) / 1000;

    printf("\nCaptured %d bars/spaces in %u ms\n", result.bar_count, result.scan_duration_ms);

    // Check if we have enough data
    if (result.bar_count < 20) {
        result.status = BC_STATUS_TOO_FEW_BARS;
        printf("ERROR: Too few bars captured (need at least 20 for CODE39)\n");
        return result;
    }

    // Calculate average narrow width and classify bars
    result.avg_narrow_width_us = calculate_avg_narrow_width(result.bars, result.bar_count);
    result.threshold_us = result.avg_narrow_width_us * 3 / 2; // 1.5x threshold

    printf("Narrow width: %u us, Wide threshold: %u us\n",
           result.avg_narrow_width_us, result.threshold_us);

    classify_bars_wide_narrow(result.bars, result.bar_count, result.threshold_us);

    // Try decoding forward
    printf("\nAttempting forward decode...\n");
    bool forward_ok = decode_code39_forward(result.bars, result.bar_count,
                                            result.data, BC_DIR_MAX_DATA_LEN);

    if (forward_ok) {
        result.scan_dir = SCAN_FORWARD;
        result.data_length = strlen(result.data);
        result.status = BC_STATUS_OK;
        printf("SUCCESS: Forward decode - \"%s\"\n", result.data);
    } else {
        // Try decoding reverse
        printf("Forward failed, attempting reverse decode...\n");
        bool reverse_ok = decode_code39_reverse(result.bars, result.bar_count,
                                                result.data, BC_DIR_MAX_DATA_LEN);

        if (reverse_ok) {
            result.scan_dir = SCAN_REVERSE;
            result.data_length = strlen(result.data);
            result.status = BC_STATUS_OK;
            printf("SUCCESS: Reverse decode - \"%s\"\n", result.data);
        } else {
            result.status = BC_STATUS_INVALID_PATTERN;
            printf("ERROR: Could not decode barcode\n");
            return result;
        }
    }

    // Determine movement direction
    result.move_dir = determine_movement_direction(result.data);

    printf("\n=== Scan Complete ===\n");
    printf("Data: \"%s\"\n", result.data);
    printf("Direction: %s\n", barcode_scan_direction_string(result.scan_dir));
    printf("Movement: %s\n", barcode_direction_string(result.move_dir));

    return result;
}

movement_direction_t barcode_get_move_direction(char c) {
    c = toupper(c);

    // Check RIGHT characters
    for (size_t i = 0; i < sizeof(RIGHT_CHARS) - 1; i++) {
        if (c == RIGHT_CHARS[i]) {
            return MOVE_RIGHT;
        }
    }

    // Check LEFT characters
    for (size_t i = 0; i < sizeof(LEFT_CHARS) - 1; i++) {
        if (c == LEFT_CHARS[i]) {
            return MOVE_LEFT;
        }
    }

    return MOVE_NONE;
}

const char* barcode_direction_string(movement_direction_t dir) {
    switch (dir) {
        case MOVE_RIGHT: return "MOVE RIGHT";
        case MOVE_LEFT:  return "MOVE LEFT";
        default:         return "NO MOVEMENT";
    }
}

const char* barcode_scan_direction_string(scan_direction_t dir) {
    switch (dir) {
        case SCAN_FORWARD: return "FORWARD (L→R)";
        case SCAN_REVERSE: return "REVERSE (R→L)";
        default:           return "UNKNOWN";
    }
}

const char* barcode_status_string(barcode_status_t status) {
    switch (status) {
        case BC_STATUS_OK:              return "OK";
        case BC_STATUS_TIMEOUT:         return "TIMEOUT";
        case BC_STATUS_NO_START:        return "NO START CHARACTER";
        case BC_STATUS_NO_STOP:         return "NO STOP CHARACTER";
        case BC_STATUS_INVALID_PATTERN: return "INVALID PATTERN";
        case BC_STATUS_TOO_FEW_BARS:    return "TOO FEW BARS";
        case BC_STATUS_CHECKSUM_ERROR:  return "CHECKSUM ERROR";
        default:                        return "UNKNOWN ERROR";
    }
}

void barcode_print_result(const barcode_result_t* result) {
    if (!result) {
        printf("NULL result\n");
        return;
    }

    printf("\n========================================\n");
    printf("BARCODE SCAN RESULT\n");
    printf("========================================\n");
    printf("Status:        %s\n", barcode_status_string(result->status));
    printf("Data:          \"%s\" (%d chars)\n", result->data, result->data_length);
    printf("Scan Dir:      %s\n", barcode_scan_direction_string(result->scan_dir));
    printf("Movement:      %s\n", barcode_direction_string(result->move_dir));
    printf("Bars Captured: %d\n", result->bar_count);
    printf("Scan Time:     %u ms\n", result->scan_duration_ms);
    printf("Narrow Width:  %u us\n", result->avg_narrow_width_us);
    printf("Threshold:     %u us\n", result->threshold_us);
    printf("========================================\n\n");
}

bool barcode_execute_movement(const barcode_result_t* result) {
    if (!result || result->status != BC_STATUS_OK) {
        return false;
    }

    switch (result->move_dir) {
        case MOVE_RIGHT:
            printf(">>> EXECUTING: Turn RIGHT\n");
            // TODO: Integrate with motor_calibration.h
            // Example: motor_turn_right();
            return true;

        case MOVE_LEFT:
            printf(">>> EXECUTING: Turn LEFT\n");
            // TODO: Integrate with motor_calibration.h
            // Example: motor_turn_left();
            return true;

        default:
            printf(">>> No movement required\n");
            return false;
    }
}

uint16_t barcode_direction_read_raw(void) {
    adc_select_input(BC_DIR_ADC_CHANNEL);
    return adc_read();
}

bool barcode_direction_read_sensor(void) {
    uint16_t value = barcode_direction_read_raw();
    return (value > adc_threshold); // true = black, false = white
}

void barcode_direction_set_threshold(uint16_t threshold) {
    adc_threshold = threshold;
    printf("ADC threshold set to: %d\n", adc_threshold);
}

uint16_t barcode_direction_get_threshold(void) {
    return adc_threshold;
}

// ==============================================================================
// Non-Blocking Continuous Scanning Implementation
// ==============================================================================

void barcode_direction_start_continuous(void) {
    continuous_scanning_enabled = true;
    continuous_scan_state = SCAN_STATE_WAITING;
    memset(&continuous_result, 0, sizeof(barcode_result_t));
    last_sensor_state_continuous = barcode_direction_read_sensor();
    last_transition_time = get_absolute_time();
    stable_white_time_us = 0;
}

void barcode_direction_stop_continuous(void) {
    continuous_scanning_enabled = false;
    continuous_scan_state = SCAN_STATE_IDLE;
}

bool barcode_direction_is_scanning(void) {
    return continuous_scanning_enabled;
}

void barcode_direction_reset(void) {
    if (continuous_scanning_enabled) {
        continuous_scan_state = SCAN_STATE_WAITING;
        memset(&continuous_result, 0, sizeof(barcode_result_t));
        last_sensor_state_continuous = barcode_direction_read_sensor();
        last_transition_time = get_absolute_time();
        stable_white_time_us = 0;
    }
}

barcode_result_t* barcode_direction_update(void) {
    if (!continuous_scanning_enabled) {
        return NULL;
    }

    bool current_state = barcode_direction_read_sensor();
    absolute_time_t current_time = get_absolute_time();

    switch (continuous_scan_state) {
        case SCAN_STATE_IDLE:
            // Should not be here if scanning is enabled
            continuous_scan_state = SCAN_STATE_WAITING;
            break;

        case SCAN_STATE_WAITING:
            // Wait for first black bar transition
            if (current_state != last_sensor_state_continuous) {
                uint32_t width_us = absolute_time_diff_us(last_transition_time, current_time);

                // Start capturing when we see a significant black bar
                if (current_state == true && width_us > BARCODE_START_MIN_US) {
                    continuous_scan_state = SCAN_STATE_CAPTURING;
                    continuous_result.bar_count = 0;
                    continuous_result.scan_duration_ms = 0;
                }

                last_transition_time = current_time;
            }
            break;

        case SCAN_STATE_CAPTURING:
            // Detect state transitions and capture bars
            if (current_state != last_sensor_state_continuous) {
                uint32_t width_us = absolute_time_diff_us(last_transition_time, current_time);

                // Record valid bars/spaces
                if (width_us >= BC_DIR_MIN_BAR_WIDTH && width_us <= BC_DIR_MAX_BAR_WIDTH) {
                    if (continuous_result.bar_count < BC_DIR_MAX_BARS) {
                        continuous_result.bars[continuous_result.bar_count].width_us = width_us;
                        continuous_result.bars[continuous_result.bar_count].is_black = last_sensor_state_continuous;
                        continuous_result.bars[continuous_result.bar_count].is_wide = false;
                        continuous_result.bar_count++;
                    }

                    stable_white_time_us = 0;
                } else if (!current_state && width_us > BARCODE_END_TIMEOUT_US) {
                    // Long white space = end of barcode
                    if (continuous_result.bar_count >= 20) {
                        continuous_scan_state = SCAN_STATE_PROCESSING;
                    } else {
                        // Too few bars, restart
                        continuous_scan_state = SCAN_STATE_WAITING;
                        continuous_result.bar_count = 0;
                    }
                }

                last_transition_time = current_time;
            } else if (!current_state) {
                // Check for end timeout during stable white
                uint32_t white_duration = absolute_time_diff_us(last_transition_time, current_time);
                if (white_duration > BARCODE_END_TIMEOUT_US && continuous_result.bar_count >= 20) {
                    continuous_scan_state = SCAN_STATE_PROCESSING;
                }
            }
            break;

        case SCAN_STATE_PROCESSING:
            // Decode the captured bars
            continuous_result.avg_narrow_width_us = calculate_avg_narrow_width(
                continuous_result.bars, continuous_result.bar_count);
            continuous_result.threshold_us = continuous_result.avg_narrow_width_us * 3 / 2;

            classify_bars_wide_narrow(continuous_result.bars,
                                      continuous_result.bar_count,
                                      continuous_result.threshold_us);

            // Try forward decode
            bool forward_ok = decode_code39_forward(continuous_result.bars,
                                                    continuous_result.bar_count,
                                                    continuous_result.data,
                                                    BC_DIR_MAX_DATA_LEN);

            if (forward_ok) {
                continuous_result.scan_dir = SCAN_FORWARD;
                continuous_result.status = BC_STATUS_OK;
            } else {
                // Try reverse decode
                bool reverse_ok = decode_code39_reverse(continuous_result.bars,
                                                       continuous_result.bar_count,
                                                       continuous_result.data,
                                                       BC_DIR_MAX_DATA_LEN);

                if (reverse_ok) {
                    continuous_result.scan_dir = SCAN_REVERSE;
                    continuous_result.status = BC_STATUS_OK;
                } else {
                    continuous_result.status = BC_STATUS_INVALID_PATTERN;
                    continuous_result.data[0] = '\0';
                }
            }

            if (continuous_result.status == BC_STATUS_OK) {
                continuous_result.data_length = strlen(continuous_result.data);
                continuous_result.move_dir = determine_movement_direction(continuous_result.data);
            } else {
                continuous_result.data_length = 0;
                continuous_result.move_dir = MOVE_NONE;
            }

            continuous_scan_state = SCAN_STATE_COMPLETE;
            break;

        case SCAN_STATE_COMPLETE:
            // Result is ready - will be returned this cycle
            // Then automatically restart scanning
            break;
    }

    last_sensor_state_continuous = current_state;

    // Return result if complete, then restart
    if (continuous_scan_state == SCAN_STATE_COMPLETE) {
        barcode_result_t* result_ptr = &continuous_result;

        // Restart scanning for next barcode
        continuous_scan_state = SCAN_STATE_WAITING;
        memset(&continuous_result, 0, sizeof(barcode_result_t));
        last_transition_time = get_absolute_time();
        stable_white_time_us = 0;

        return result_ptr;
    }

    return NULL;
}

// ==============================================================================
// Private Helper Functions
// ==============================================================================

static uint32_t calculate_avg_narrow_width(bar_element_t* bars, uint16_t count) {
    uint32_t sum = 0;
    uint16_t narrow_count = 0;

    // Find smallest ~60% of bars (likely narrow bars)
    for (uint16_t i = 0; i < count; i++) {
        if (bars[i].width_us < 500) { // Reasonable upper bound for narrow
            sum += bars[i].width_us;
            narrow_count++;
        }
    }

    if (narrow_count > 0) {
        return sum / narrow_count;
    }

    return 150; // Default fallback
}

static void classify_bars_wide_narrow(bar_element_t* bars, uint16_t count, uint32_t threshold) {
    for (uint16_t i = 0; i < count; i++) {
        bars[i].is_wide = (bars[i].width_us > threshold);
    }
}

static char decode_code39_character(uint8_t* pattern) {
    for (size_t i = 0; i < CODE39_TABLE_SIZE; i++) {
        if (match_pattern(pattern, (uint8_t*)CODE39_TABLE[i].pattern, 9)) {
            return CODE39_TABLE[i].character;
        }
    }
    return '\0'; // No match
}

static bool match_pattern(uint8_t* p1, uint8_t* p2, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (p1[i] != p2[i]) {
            return false;
        }
    }
    return true;
}

static bool decode_code39_forward(bar_element_t* bars, uint16_t count, char* output, uint8_t max_len) {
    uint8_t pattern[9];
    uint8_t data_idx = 0;
    bool found_start = false;

    // Scan through bars looking for characters
    for (uint16_t i = 0; i + 8 < count; i++) {
        // Extract 9-element pattern (bars and spaces alternate)
        for (uint8_t j = 0; j < 9; j++) {
            pattern[j] = bars[i + j].is_wide ? 1 : 0;
        }

        char decoded = decode_code39_character(pattern);

        if (decoded == CODE39_START_STOP && !found_start) {
            // Found start character
            found_start = true;
            i += 8; // Skip past start character
            continue;
        } else if (decoded == CODE39_START_STOP && found_start) {
            // Found stop character - success!
            output[data_idx] = '\0';
            return (data_idx > 0);
        } else if (found_start && decoded != '\0') {
            // Valid data character
            if (data_idx < max_len) {
                output[data_idx++] = decoded;
                i += 8; // Skip to next character
            }
        }
    }

    return false; // No valid barcode found
}

static bool decode_code39_reverse(bar_element_t* bars, uint16_t count, char* output, uint8_t max_len) {
    uint8_t pattern[9];
    uint8_t data_idx = 0;
    bool found_start = false;

    // Scan backwards through bars
    for (int i = count - 9; i >= 0; i--) {
        // Extract 9-element pattern in reverse
        for (uint8_t j = 0; j < 9; j++) {
            pattern[8 - j] = bars[i + j].is_wide ? 1 : 0;
        }

        char decoded = decode_code39_character(pattern);

        if (decoded == CODE39_START_STOP && !found_start) {
            // Found start character (reading backwards)
            found_start = true;
            i -= 8;
            continue;
        } else if (decoded == CODE39_START_STOP && found_start) {
            // Found stop character - success!
            output[data_idx] = '\0';
            reverse_string(output); // Reverse the string since we read backwards
            return (data_idx > 0);
        } else if (found_start && decoded != '\0') {
            // Valid data character
            if (data_idx < max_len) {
                output[data_idx++] = decoded;
                i -= 8;
            }
        }
    }

    return false;
}

static void reverse_string(char* str) {
    if (!str) return;

    size_t len = strlen(str);
    for (size_t i = 0; i < len / 2; i++) {
        char temp = str[i];
        str[i] = str[len - 1 - i];
        str[len - 1 - i] = temp;
    }
}

static movement_direction_t determine_movement_direction(const char* data) {
    if (!data || strlen(data) == 0) {
        return MOVE_NONE;
    }

    // Use the first valid character to determine direction
    for (size_t i = 0; i < strlen(data); i++) {
        movement_direction_t dir = barcode_get_move_direction(data[i]);
        if (dir != MOVE_NONE) {
            return dir;
        }
    }

    return MOVE_NONE;
}
