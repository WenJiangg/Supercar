# # CMakeLists.txt for Barcode Decoder
# cmake_minimum_required(VERSION 3.13)

# # Include the Pico SDK
# include(pico_sdk_import.cmake)

# # Project name
# project(barcode_decoder C CXX ASM)
# set(CMAKE_C_STANDARD 11)
# set(CMAKE_CXX_STANDARD 17)

# # Initialize the Pico SDK
# pico_sdk_init()

# # Add the barcode decoder library
# add_library(barcode_decoder
#     barcode_decoder.c
# )

# target_link_libraries(barcode_decoder
#     pico_stdlib
#     hardware_gpio
#     hardware_timer
#     hardware_adc
# )

# # Add the test executable
# add_executable(barcode_test
#     barcode_test.c
# )

# target_link_libraries(barcode_test
#     barcode_decoder
#     pico_stdlib
# )

# # Enable USB output, disable UART output
# pico_enable_stdio_usb(barcode_test 1)
# pico_enable_stdio_uart(barcode_test 0)

# # Create map/bin/hex/uf2 file etc.
# pico_add_extra_outputs(barcode_test)


cmake_minimum_required(VERSION 3.13)

# Use the "import" file, which is correct for
# projects nested inside pico-examples
include(pico_sdk_import.cmake)

# Define the project
project(RoboCar C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK (required after import)
pico_sdk_init()

# ==============================================================================
#  EXECUTABLE TARGET
# ==============================================================================

# Define the executable (RoboCar) and list ALL your .c source files
add_executable(RoboCar
    main.c
    # obstacle_detection.c  <-- REMOVED
    ir_sensor.c
    robot_controller.c
    encoder.c
    imu.c
    servo_control.c       # <-- ADDED
    ultrasonic_sensor.c   # <-- ADDED
)

# Add the current directory so main.c can find headers
target_include_directories(RoboCar PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# ==============================================================================
#  LINK LIBRARIES
# ==============================================================================

# Link all the libraries your project needs
target_link_libraries(RoboCar PRIVATE
    pico_stdlib
    hardware_pwm    # For the servo and motors
    hardware_gpio   # For servo and ultrasonic
    hardware_adc    # For the IR line sensor
    m               # For math (sqrtf, cosf)
    hardware_i2c
    hardware_sync

    # --- Use the Manual Polling Model ---
    pico_cyw43_arch_lwip_poll
    pico_lwip_mqtt
)

# ==============================================================================
#  OUTPUT & STDIO
# ==============================================================================

# Create the .uf2 file
pico_add_extra_outputs(RoboCar)

# Enable printf() output over USB
pico_enable_stdio_usb(RoboCar 1)

# Enable printf() output over UART
pico_enable_stdio_uart(RoboCar 1)