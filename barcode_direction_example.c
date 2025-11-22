/**
 * ==============================================================================
 * BARCODE_DIRECTION_EXAMPLE.C - Example Usage of Barcode Direction Detection
 * ==============================================================================
 *
 * This file demonstrates how to use the barcode_direction module to:
 * 1. Initialize and calibrate the barcode sensor
 * 2. Scan CODE39 barcodes
 * 3. Detect scan direction (forward/reverse)
 * 4. Determine movement direction based on decoded characters
 * 5. Integrate with motor control
 *
 * Hardware Setup:
 * - IR Sensor (TCRT5000 or similar) on GPIO28 (ADC2)
 * - Sensor should be 3-8mm from barcode surface
 *
 * ==============================================================================
 */

#include "pico/stdlib.h"
#include "barcode_direction.h"
#include "motor_calibration.h"  // For actual motor control
#include <stdio.h>

// ==============================================================================
// Example 1: Basic Barcode Scanning
// ==============================================================================

void example_basic_scan(void) {
    printf("\n=== EXAMPLE 1: Basic Barcode Scanning ===\n\n");

    // Scan a barcode with 5 second timeout
    barcode_result_t result = barcode_direction_scan(5000);

    // Check if scan was successful
    if (result.status == BC_STATUS_OK) {
        printf("Scan successful!\n");
        printf("Decoded: %s\n", result.data);
        printf("Direction: %s\n", barcode_scan_direction_string(result.scan_dir));
        printf("Movement: %s\n", barcode_direction_string(result.move_dir));
    } else {
        printf("Scan failed: %s\n", barcode_status_string(result.status));
    }

    // Print detailed result
    barcode_print_result(&result);
}

// ==============================================================================
// Example 2: Continuous Scanning Loop
// ==============================================================================

void example_continuous_scan(void) {
    printf("\n=== EXAMPLE 2: Continuous Scanning ===\n");
    printf("Press Ctrl+C to stop...\n\n");

    uint32_t scan_count = 0;

    while (true) {
        printf("\n--- Scan #%lu ---\n", ++scan_count);

        barcode_result_t result = barcode_direction_scan(10000);

        if (result.status == BC_STATUS_OK) {
            printf("SUCCESS: \"%s\" → %s\n",
                   result.data,
                   barcode_direction_string(result.move_dir));
        } else {
            printf("FAILED: %s\n", barcode_status_string(result.status));
        }

        sleep_ms(1000); // Small delay between scans
    }
}

// ==============================================================================
// Example 3: Scan and Execute Movement
// ==============================================================================

void example_scan_and_move(void) {
    printf("\n=== EXAMPLE 3: Scan and Execute Movement ===\n\n");

    // Initialize motor system (if not already done)
    // motor_init(); // Uncomment if motors not initialized in main

    barcode_result_t result = barcode_direction_scan(5000);

    if (result.status == BC_STATUS_OK) {
        printf("Barcode: \"%s\"\n", result.data);

        // Execute movement based on the first character
        switch (result.move_dir) {
            case MOVE_RIGHT:
                printf(">>> Turning RIGHT\n");
                // Example motor commands (adjust to your setup):
                // motor_set_differential(-30, 30);  // Turn right
                // sleep_ms(500);
                // motor_stop();
                break;

            case MOVE_LEFT:
                printf(">>> Turning LEFT\n");
                // Example motor commands (adjust to your setup):
                // motor_set_differential(30, -30);  // Turn left
                // sleep_ms(500);
                // motor_stop();
                break;

            default:
                printf(">>> No movement required\n");
                break;
        }
    } else {
        printf("Scan failed: %s\n", barcode_status_string(result.status));
    }
}

// ==============================================================================
// Example 4: Character-by-Character Processing
// ==============================================================================

void example_process_each_character(void) {
    printf("\n=== EXAMPLE 4: Process Each Character ===\n\n");

    barcode_result_t result = barcode_direction_scan(5000);

    if (result.status == BC_STATUS_OK) {
        printf("Barcode data: \"%s\"\n", result.data);
        printf("\nProcessing each character:\n");

        for (uint8_t i = 0; i < result.data_length; i++) {
            char c = result.data[i];
            movement_direction_t dir = barcode_get_move_direction(c);

            printf("  [%d] '%c' → %s\n", i, c, barcode_direction_string(dir));

            // You could execute a movement for each character here
            // For example, create a sequence of movements
        }
    } else {
        printf("Scan failed: %s\n", barcode_status_string(result.status));
    }
}

// ==============================================================================
// Example 5: Test All Movement Characters
// ==============================================================================

void example_test_all_characters(void) {
    printf("\n=== EXAMPLE 5: Test All Movement Characters ===\n\n");

    printf("RIGHT movement characters:\n");
    const char* right_chars = "ACEGIKMOQSUWY";
    for (size_t i = 0; right_chars[i] != '\0'; i++) {
        char c = right_chars[i];
        movement_direction_t dir = barcode_get_move_direction(c);
        printf("  %c → %s\n", c, barcode_direction_string(dir));
    }

    printf("\nLEFT movement characters:\n");
    const char* left_chars = "BDFHJLNPRTVXZ";
    for (size_t i = 0; left_chars[i] != '\0'; i++) {
        char c = left_chars[i];
        movement_direction_t dir = barcode_get_move_direction(c);
        printf("  %c → %s\n", c, barcode_direction_string(dir));
    }

    printf("\nOther characters (numbers, symbols):\n");
    const char* other_chars = "0123456789-. $";
    for (size_t i = 0; other_chars[i] != '\0'; i++) {
        char c = other_chars[i];
        movement_direction_t dir = barcode_get_move_direction(c);
        printf("  %c → %s\n", c, barcode_direction_string(dir));
    }
}

// ==============================================================================
// Example 6: Real-time Sensor Reading
// ==============================================================================

void example_realtime_sensor(void) {
    printf("\n=== EXAMPLE 6: Real-time Sensor Reading ===\n");
    printf("Move barcode under sensor to see real-time ADC values\n");
    printf("Press any key to stop...\n\n");

    // Clear input buffer
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);

    uint32_t count = 0;
    while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT) {
        uint16_t raw = barcode_direction_read_raw();
        bool state = barcode_direction_read_sensor();

        if (count % 10 == 0) {
            printf("\r[%6lu] ADC: %4d | State: %s | Threshold: %d     ",
                   count,
                   raw,
                   state ? "BLACK" : "WHITE",
                   barcode_direction_get_threshold());
            fflush(stdout);
        }

        count++;
        sleep_ms(50);
    }

    printf("\n");
}

// ==============================================================================
// Main Example Menu
// ==============================================================================

void run_example_menu(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║   CODE39 Barcode Direction Detection Examples         ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Select an example to run:\n");
    printf("  1. Basic Barcode Scanning\n");
    printf("  2. Continuous Scanning Loop\n");
    printf("  3. Scan and Execute Movement\n");
    printf("  4. Character-by-Character Processing\n");
    printf("  5. Test All Movement Characters\n");
    printf("  6. Real-time Sensor Reading\n");
    printf("  C. Calibrate Sensor\n");
    printf("  Q. Quit\n");
    printf("\n");
    printf("Enter choice: ");
    fflush(stdout);

    int choice = getchar();
    printf("\n");

    switch (choice) {
        case '1': example_basic_scan(); break;
        case '2': example_continuous_scan(); break;
        case '3': example_scan_and_move(); break;
        case '4': example_process_each_character(); break;
        case '5': example_test_all_characters(); break;
        case '6': example_realtime_sensor(); break;
        case 'C':
        case 'c':
            barcode_direction_calibrate();
            break;
        case 'Q':
        case 'q':
            printf("Exiting...\n");
            return;
        default:
            printf("Invalid choice\n");
            break;
    }

    printf("\nPress any key to return to menu...");
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    getchar();
}

// ==============================================================================
// Main Function (if used as standalone program)
// ==============================================================================

#ifdef BARCODE_DIRECTION_STANDALONE

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial

    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║  CODE39 Barcode Direction Detection System            ║\n");
    printf("║  Hardware: GPIO28 (ADC2)                               ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    printf("\n");

    // Initialize the barcode direction system
    barcode_direction_init();

    // Optional: Run calibration at startup
    printf("Do you want to calibrate the sensor? (y/n): ");
    fflush(stdout);
    int response = getchar();
    if (response == 'y' || response == 'Y') {
        barcode_direction_calibrate();
    }

    // Run example menu in a loop
    while (true) {
        run_example_menu();
    }

    return 0;
}

#endif // BARCODE_DIRECTION_STANDALONE

// ==============================================================================
// Integration Example with Robot Controller
// ==============================================================================

/**
 * @brief Example function showing integration with robot_controller.c
 *
 * This would typically be called from your main robot control loop
 * when you want to process a barcode instruction.
 */
void integrate_with_robot_controller_example(void) {
    // This is how you might integrate barcode scanning into your robot:

    // 1. Initialize in main.c or robot_controller_init()
    barcode_direction_init();

    // 2. In your control loop, trigger barcode scan when needed
    // (e.g., when robot reaches a decision point)

    printf("Robot reached decision point - scanning barcode...\n");

    barcode_result_t result = barcode_direction_scan(3000);

    if (result.status == BC_STATUS_OK) {
        printf("Barcode instruction: %s\n", result.data);
        printf("Scan direction: %s\n", barcode_scan_direction_string(result.scan_dir));

        // Execute movement based on first character
        switch (result.move_dir) {
            case MOVE_RIGHT:
                printf("Executing RIGHT turn...\n");
                // Call your motor control functions:
                // motor_set_differential(-40, 40);  // Turn right
                // sleep_ms(800);  // Turn duration
                // motor_stop();
                break;

            case MOVE_LEFT:
                printf("Executing LEFT turn...\n");
                // motor_set_differential(40, -40);  // Turn left
                // sleep_ms(800);
                // motor_stop();
                break;

            default:
                printf("No movement instruction\n");
                break;
        }
    } else {
        printf("Barcode scan failed: %s\n", barcode_status_string(result.status));
        // Handle error (e.g., retry, continue straight, etc.)
    }
}
