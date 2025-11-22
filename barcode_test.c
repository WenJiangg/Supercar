/**
 * ==============================================================================
 * BARCODE_TEST.C - Test Program for Barcode Decoder
 * ==============================================================================
 * This demonstrates how to use the barcode decoder with the IR sensor on GPIO3
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "barcode_decoder.h"

int main() {
    // Initialize stdio for USB serial communication
    stdio_init_all();
    
    // Wait a bit for USB serial to be ready
    sleep_ms(2000);
    
    printf("\n");
    printf("========================================\n");
    printf("  BARCODE DECODER TEST - GPIO28\n");
    printf("========================================\n");
    printf("Raspberry Pi Pico W\n");
    printf("TCRT5000 on GP28 (ADC2) - Analog Mode\n");
    printf("\n");
    
    // Initialize barcode decoder
    barcode_init();
    
    printf("\nBarcode decoder ready!\n");
    printf("Available commands:\n");
    printf("  1 - Scan barcode (blocking)\n");
    printf("  2 - Scan barcode (non-blocking)\n");
    printf("  3 - Read raw sensor value\n");
    printf("  4 - Continuous sensor monitor\n");
    printf("  5 - Calibrate sensor (IMPORTANT - Do this first!)\n");
    printf("  6 - Monitor ADC values\n");
    printf("  q - Quit\n");
    printf("\n");
    printf("⚠️  TIP: Run command 5 first to calibrate your sensor!\n");
    printf("\n");
    
    char cmd;
    bool running = true;
    
    while (running) {
        printf("> ");
        fflush(stdout);
        
        // Wait for input
        while ((cmd = getchar_timeout_us(100000)) == PICO_ERROR_TIMEOUT) {
            tight_loop_contents();
        }
        
        printf("%c\n", cmd);
        
        switch (cmd) {
            case '1': {
                // Blocking scan
                printf("\n--- Blocking Barcode Scan ---\n");
                barcode_t bc = barcode_scan(10000);  // 10 second timeout
                
                if (bc.valid) {
                    barcode_print_info(&bc);
                    printf("✓ SUCCESS - Decoded: \"%s\"\n", bc.data);
                } else {
                    printf("✗ FAILED - No valid barcode detected\n");
                }
                printf("\n");
                break;
            }
            
            case '2': {
                // Non-blocking scan
                printf("\n--- Non-blocking Barcode Scan ---\n");
                printf("Move barcode under sensor...\n");
                printf("Press any key to cancel\n");
                
                if (barcode_scan_start()) {
                    bool cancelled = false;
                    barcode_t* result = NULL;
                    
                    while ((result = barcode_scan_check()) == NULL) {
                        // Check for user cancellation
                        if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
                            cancelled = true;
                            barcode_scan_cancel();
                            break;
                        }
                        sleep_ms(10);
                    }
                    
                    if (cancelled) {
                        printf("Scan cancelled by user\n");
                    } else if (result && result->valid) {
                        barcode_print_info(result);
                        printf("✓ SUCCESS - Decoded: \"%s\"\n", result->data);
                    } else {
                        printf("✗ FAILED - No valid barcode detected\n");
                    }
                }
                printf("\n");
                break;
            }
            
            case '3': {
                // Read raw sensor
                printf("\nReading raw sensor value...\n");
                for (int i = 0; i < 10; i++) {
                    uint16_t raw = barcode_read_raw();
                    bool value = barcode_read_sensor();
                    printf("  ADC: %4d | %s\n", raw, value ? "BLACK (bar)" : "WHITE (space)");
                    sleep_ms(200);
                }
                printf("\n");
                break;
            }
            
            case '4': {
                // Continuous monitor
                printf("\n--- Continuous Sensor Monitor ---\n");
                printf("Press any key to stop\n\n");
                
                bool last_state = barcode_read_sensor();
                uint32_t transition_count = 0;
                absolute_time_t last_transition = get_absolute_time();
                
                while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT) {
                    bool current_state = barcode_read_sensor();
                    
                    if (current_state != last_state) {
                        absolute_time_t now = get_absolute_time();
                        uint32_t width_us = absolute_time_diff_us(last_transition, now);
                        
                        printf("[%6lu us] %s -> %s\n", 
                               width_us,
                               last_state ? "BLACK" : "WHITE",
                               current_state ? "BLACK" : "WHITE");
                        
                        transition_count++;
                        last_state = current_state;
                        last_transition = now;
                    }
                    
                    sleep_us(50);
                }
                
                printf("\nTotal transitions: %u\n", transition_count);
                // Clear input buffer
                while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
                printf("\n");
                break;
            }
            
            case '5': {
                // Calibrate sensor
                printf("\n--- Sensor Calibration ---\n");
                barcode_calibrate_sensor();
                printf("\n");
                break;
            }
            
            case '6': {
                // Monitor ADC continuously
                printf("\n--- ADC Monitor Mode ---\n");
                printf("Press any key to stop\n\n");
                
                uint16_t min_val = 4095;
                uint16_t max_val = 0;
                
                while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT) {
                    uint16_t value = barcode_read_raw();
                    bool is_black = barcode_read_sensor();
                    
                    if (value < min_val) min_val = value;
                    if (value > max_val) max_val = value;
                    
                    printf("ADC: %4d | %s | Min: %4d, Max: %4d\r", 
                           value,
                           is_black ? "BLACK ████" : "WHITE     ",
                           min_val, max_val);
                    fflush(stdout);
                    
                    sleep_ms(50);
                }
                
                printf("\n\n");
                // Clear input buffer
                while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
                break;
            }
            
            case 'q':
            case 'Q':
                printf("\nExiting...\n");
                running = false;
                break;
                
            default:
                printf("Unknown command\n\n");
                break;
        }
    }
    
    printf("Goodbye!\n");
    return 0;
}