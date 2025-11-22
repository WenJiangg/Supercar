/**
 * ==============================================================================
 * MAIN.C - RoboCar Entry Point
 * ==============================================================================
 * Responsibilities:
 * 1. Initialize stdio
 * 2. Initialize WiFi and MQTT
 * 3. Initialize ALL hardware modules (Motors, Encoders, IR, IMU, Servo, Ultra)
 * 4. Call the robot_controller_init()
 * 5. Call the robot_controller_run_loop()
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

// Core Modules
#include "mqtt_handler.h"
#include "robot_controller.h"

// Hardware Modules
#include "motor_calibration.h"
#include "ir_sensor.h"
#include "encoder.h"
#include "imu.h"
#include "servo_control.h"      // <-- ADDED
#include "ultrasonic_sensor.h"  // <-- ADDED


int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial to connect

    // 1. Init WiFi
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_SINGAPORE)) {
        printf("Failed to init cyw43\n"); return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("cyw43 initialized\n");

    // 2. Init WiFi connection
    if (!wifi_connect()) {
        printf("Failed to connect to WiFi\n"); return -1;
    }

    // 3. Init MQTT
    if (!mqtt_init()) {
        printf("Failed to connect to MQTT\n"); return -1;
    }
    robot_log("Network OK. Initializing robot hardware...\n");

    // 4. Init ALL Hardware Modules
    motor_init();
    ir_sensor_init();
    encoder_init();
    
    // --- NEW MODULES ADDED ---
    servo_init();
    ultrasonic_init();
    // -------------------------
    
    if (!imu_init()) {
        robot_log("WARNING: IMU not found! Continuing without IMU...\n");
    }

    // 5. Init the Robot Controller (new function)
    // This sets up the master state machine
    robot_controller_init();

    // 6. Run the Controller's Infinite Loop
    // This function now contains the while(true) loop
    robot_controller_run_loop();

    return 0; // Should never be reached
}