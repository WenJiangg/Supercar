# CODE39 Barcode Direction Detection System

## Overview

This module provides comprehensive CODE39 barcode decoding with automatic direction detection and movement command generation for the RoboCar project.

## Features

- ✅ **Single IR Sensor Decoding** - Uses only GPIO28 (ADC2)
- ✅ **CODE39 Format Support** - Full CODE39 character set (A-Z, 0-9, symbols)
- ✅ **Wide/Narrow Bar Classification** - Automatic threshold calculation
- ✅ **Bidirectional Scanning** - Detects and handles forward/reverse scans
- ✅ **Movement Direction Mapping** - Automatic LEFT/RIGHT commands based on characters
- ✅ **Easy Calibration** - Interactive white/black calibration routine
- ✅ **Continuous Non-Blocking Scanning** - Scans while robot is line following
- ✅ **MQTT Integration** - Automatic publishing of barcode results
- ✅ **Real-time Detection** - Updates every 2ms without blocking movement

## Hardware Setup

### Required Components
- **IR Sensor**: TCRT5000 or similar reflective IR sensor
- **GPIO Pin**: GPIO28 (ADC2 on Raspberry Pi Pico)
- **Distance**: 3-8mm from barcode surface for optimal reading

### Wiring
```
TCRT5000 Sensor → Raspberry Pi Pico
─────────────────────────────────────
VCC             → 3.3V or 5V
GND             → GND
A0 (Analog)     → GPIO28 (ADC2)
```

## Movement Direction Rules

### Move RIGHT
Characters: **A, C, E, G, I, K, M, O, Q, S, U, W, Y**

### Move LEFT
Characters: **B, D, F, H, J, L, N, P, R, T, V, X, Z**

### No Movement
Numbers (0-9) and special characters (-, ., $, /, +, %, space)

## Quick Start Guide

### 1. Include Headers

```c
#include "barcode_direction.h"
```

### 2. Initialize System

```c
int main() {
    stdio_init_all();

    // Initialize barcode system
    barcode_direction_init();

    // Optional: Calibrate sensor
    barcode_direction_calibrate();

    // Your code here...
}
```

### 3. Scan Barcode

```c
// Scan with 5 second timeout
barcode_result_t result = barcode_direction_scan(5000);

if (result.status == BC_STATUS_OK) {
    printf("Decoded: %s\n", result.data);
    printf("Movement: %s\n", barcode_direction_string(result.move_dir));

    // Execute movement
    if (result.move_dir == MOVE_RIGHT) {
        // Turn right
    } else if (result.move_dir == MOVE_LEFT) {
        // Turn left
    }
}
```

## API Reference

### Initialization Functions

#### `void barcode_direction_init(void)`
Initializes the barcode detection system. Must be called before any other functions.

#### `void barcode_direction_calibrate(void)`
Interactive calibration routine. Guides user through white/black surface calibration.

### Scanning Functions

#### `barcode_result_t barcode_direction_scan(uint32_t timeout_ms)`
Scans and decodes a CODE39 barcode.

**Parameters:**
- `timeout_ms` - Maximum time to wait (0 = no timeout)

**Returns:** `barcode_result_t` structure with:
- `data` - Decoded string (null-terminated)
- `data_length` - Number of characters
- `scan_dir` - Scan direction (FORWARD/REVERSE)
- `move_dir` - Movement command (LEFT/RIGHT/NONE)
- `status` - Decode status
- `bar_count` - Number of bars captured
- `scan_duration_ms` - Scan time

### Direction Functions

#### `movement_direction_t barcode_get_move_direction(char c)`
Gets movement direction for a specific character.

**Parameters:**
- `c` - Character to check

**Returns:** `MOVE_RIGHT`, `MOVE_LEFT`, or `MOVE_NONE`

### Utility Functions

#### `void barcode_print_result(const barcode_result_t* result)`
Prints detailed scan result information.

#### `uint16_t barcode_direction_read_raw(void)`
Reads raw ADC value (0-4095).

#### `bool barcode_direction_read_sensor(void)`
Reads sensor state (true = black, false = white).

#### `void barcode_direction_set_threshold(uint16_t threshold)`
Sets ADC threshold for black/white detection.

## Continuous Scanning Mode (Recommended for RoboCar)

The barcode scanner is integrated into the robot controller and runs continuously while your car is line following. This mode is **automatically enabled** when you initialize the robot controller.

### How It Works

1. **Non-Blocking** - Scanner runs in background, never blocks PID control
2. **Automatic Detection** - Detects barcodes as they pass under GPIO28 sensor
3. **MQTT Publishing** - Results automatically published to `robocar/barcode` topic
4. **Auto-Reset** - After detecting a barcode, automatically starts scanning for the next one

### Integration (Already Done in robot_controller.c)

```c
// In robot_controller_init():
barcode_direction_init();
barcode_direction_start_continuous();

// In robot_controller_run_loop() (called every 2ms):
barcode_result_t* result = barcode_direction_update();
if (result != NULL && result->status == BC_STATUS_OK) {
    // Barcode detected!
    printf("Barcode: %s | %s\n",
           result->data,
           barcode_direction_string(result->move_dir));

    // Publish to MQTT
    // ... (automatic in robot_controller.c)
}
```

### MQTT Message Format

When a barcode is detected, the following JSON is published to `robocar/barcode`:

```json
{
    "barcode": "ABC",
    "direction": "MOVE RIGHT",
    "scan_dir": "FORWARD (L→R)",
    "move_dir": 1,
    "bar_count": 45
}
```

**Fields:**
- `barcode` - Decoded string
- `direction` - Human-readable movement command
- `scan_dir` - Scan direction (forward/reverse)
- `move_dir` - Integer: 0=NONE, 1=RIGHT, 2=LEFT
- `bar_count` - Number of bars captured

### Manual Control

If you want to manually control continuous scanning:

```c
// Start continuous scanning
barcode_direction_start_continuous();

// In your main loop
barcode_result_t* result = barcode_direction_update();
if (result != NULL) {
    // Process result
}

// Stop continuous scanning
barcode_direction_stop_continuous();

// Reset scanner (clear current scan data)
barcode_direction_reset();

// Check if scanning is active
if (barcode_direction_is_scanning()) {
    // Scanning is active
}
```

## Usage Examples

### Example 1: Basic Scan (Blocking Mode)

```c
barcode_result_t result = barcode_direction_scan(5000);

if (result.status == BC_STATUS_OK) {
    printf("Barcode: %s\n", result.data);
} else {
    printf("Error: %s\n", barcode_status_string(result.status));
}
```

### Example 2: Process Each Character

```c
barcode_result_t result = barcode_direction_scan(5000);

if (result.status == BC_STATUS_OK) {
    for (uint8_t i = 0; i < result.data_length; i++) {
        char c = result.data[i];
        movement_direction_t dir = barcode_get_move_direction(c);

        printf("'%c' → %s\n", c, barcode_direction_string(dir));

        // Execute movement for each character
        if (dir == MOVE_RIGHT) {
            // Turn right
        } else if (dir == MOVE_LEFT) {
            // Turn left
        }
    }
}
```

### Example 3: Integration with Motor Control

```c
void execute_barcode_command(void) {
    barcode_result_t result = barcode_direction_scan(3000);

    if (result.status == BC_STATUS_OK) {
        printf("Instruction: %s → %s\n",
               result.data,
               barcode_direction_string(result.move_dir));

        switch (result.move_dir) {
            case MOVE_RIGHT:
                motor_set_differential(-40, 40);  // Turn right
                sleep_ms(800);
                motor_stop();
                break;

            case MOVE_LEFT:
                motor_set_differential(40, -40);  // Turn left
                sleep_ms(800);
                motor_stop();
                break;

            default:
                // Continue straight or no action
                break;
        }
    }
}
```

## Calibration Guide

### Why Calibrate?

Different sensors and surfaces can have varying reflectance values. Calibration ensures optimal black/white detection.

### Calibration Steps

1. Call `barcode_direction_calibrate()`
2. Place sensor over **WHITE** paper when prompted
3. Press any key to sample white values
4. Place sensor over **BLACK** barcode bar when prompted
5. Press any key to sample black values
6. System automatically calculates optimal threshold

### Expected Values

- **Good range**: Black - White > 500
- **Typical white**: 200-800
- **Typical black**: 2500-3800
- **Auto-calculated threshold**: (White + Black) / 2

## Troubleshooting

### Problem: "Too few bars captured"

**Solutions:**
- Ensure barcode is CODE39 format
- Check sensor distance (3-8mm optimal)
- Verify barcode quality (clear, high contrast)
- Recalibrate sensor

### Problem: "Invalid pattern"

**Solutions:**
- Scan barcode slower for better sampling
- Check barcode format (must be CODE39)
- Verify start/stop characters (*) are present
- Try scanning in reverse direction

### Problem: Poor sensor readings

**Solutions:**
- Run calibration routine
- Check sensor wiring
- Adjust sensor distance to barcode
- Ensure adequate lighting (IR sensors prefer darker environments)
- Clean sensor lens

### Problem: Intermittent readings

**Solutions:**
- Stabilize sensor mount
- Ensure consistent scan speed
- Check power supply stability
- Verify ADC reference voltage

## CODE39 Barcode Format

### Structure
```
*DATA*
```
- Start character: `*`
- Data: Any valid CODE39 characters
- Stop character: `*`

### Valid Characters
- **Letters**: A-Z (uppercase only)
- **Numbers**: 0-9
- **Special**: `-`, `.`, ` `, `$`, `/`, `+`, `%`

### Example Barcodes

| Barcode | Decoded | Movement |
|---------|---------|----------|
| `*A*` | A | RIGHT |
| `*B*` | B | LEFT |
| `*GO*` | GO | RIGHT (G) |
| `*STOP*` | STOP | RIGHT (S) |
| `*LEFT*` | LEFT | LEFT (L) |
| `*ABC*` | ABC | RIGHT (A) |

## Technical Details

### Bar Width Classification

The system uses adaptive threshold calculation:

1. **Capture all bars** during scan
2. **Calculate average narrow width** from smallest bars
3. **Set threshold** at 1.5× average narrow width
4. **Classify bars**: width > threshold = WIDE, else NARROW

### Direction Detection Algorithm

1. **Forward Decode**:
   - Scan left-to-right
   - Look for start `*`, decode characters, look for stop `*`

2. **Reverse Decode** (if forward fails):
   - Scan right-to-left
   - Reverse pattern matching
   - Reverse decoded string

3. **Result**: Automatically selects successful direction

### Performance

- **Scan speed**: 50-500 mm/s (depending on bar width)
- **Typical scan time**: 100-2000 ms
- **Bar width range**: 50 μs - 10 ms
- **Maximum bars**: 100 elements
- **Maximum data**: 30 characters

## Files

### Core Module
- `barcode_direction.h` - Header file with API definitions
- `barcode_direction.c` - Implementation with CODE39 decoder

### Examples
- `barcode_direction_example.c` - Comprehensive usage examples
- `BARCODE_DIRECTION_README.md` - This documentation

### Integration
- Add to `CMakeLists.txt`:
  ```cmake
  add_executable(RoboCar
      # ... other files ...
      barcode_direction.c
  )
  ```

## License

Part of the RoboCar project. See main project README for license information.

## Support

For issues or questions:
1. Check troubleshooting section above
2. Verify hardware connections
3. Run calibration routine
4. Review example code
5. Check sensor datasheet for your specific model

## Future Enhancements

- [ ] Support for other barcode formats (Code128, EAN, etc.)
- [ ] Checksum validation
- [ ] Multi-character movement sequences
- [ ] MQTT integration for remote barcode commands
- [ ] Barcode database lookup
- [ ] Visual feedback (LED indicators)

---

**Version**: 1.0
**Last Updated**: 2025-11-22
**Hardware**: Raspberry Pi Pico + TCRT5000
**Format**: CODE39
