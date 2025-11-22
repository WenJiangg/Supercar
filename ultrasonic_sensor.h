#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "pico/types.h" 

// Pins (unchanged)
#define ULTRASONIC_TRIG_PIN     16   // GP16
#define ULTRASONIC_ECHO_PIN     17   // GP17

// Timing (fast but safe)
#define US_MIN_VALID_US        120u      // <~2 cm (ringing)
#define US_MAX_VALID_US      20000u      // ~3.4 m timeout
#define US_SERVO_SETTLE_US  150000u      // 150 ms after each servo move
#define US_PING_SPACING_MS      60u      // >=60 ms between pings

// Median filter size (3 or 5)
#define US_FILTER_COUNT          3u

typedef enum {
    US_OK = 0,
    US_TIMEOUT = -1,
    US_INVALID_TOO_CLOSE = -2,
    US_NOT_READY = -3
} us_status_t;

// Init + non-blocking read API
void ultrasonic_init(void);               // config TRIG/ECHO, enable ECHO IRQ (no global callback)
void ultrasonic_start_read(void);         // start a ping (non-blocking)
bool ultrasonic_update(void);             // progress; true when finished
us_status_t ultrasonic_get_last_reading_cm(float *cm_out);

// Helpers for integration with servo scheduling
void ultrasonic_mark_servo_moved(uint32_t now_us);
bool ultrasonic_servo_settled(uint32_t now_us);
bool ultrasonic_ping_spacing_ok(uint32_t now_us);

// NEW: router entry called from the single global GPIO ISR
void ultrasonic_on_gpio_event(uint gpio, uint32_t events);

// (Optional) convenience if your code wants last distance without status
static inline float ultrasonic_get_last_distance(void){
    float cm=0.0f; return (ultrasonic_get_last_reading_cm(&cm)==US_OK)? cm : -1.0f;
}
