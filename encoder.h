/**
 * ==============================================================================
 * ENCODER.H — Single global ISR (encoders) with ultrasonic routing
 * ==============================================================================
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

// ----- Pins (keep as in your project) -----
#ifndef ENC_LEFT_PIN
#define ENC_LEFT_PIN   26
#endif
#ifndef ENC_RIGHT_PIN
#define ENC_RIGHT_PIN   5
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Init + filters
void encoder_init(void);
void encoder_reset_glitch_filter(void);

// Periodic kinematics update (optional if you want time-window math)
void encoder_update_kinematics(void);

// Speeds / RPM / distance
float encoder_get_left_speed_cm_s(void);
float encoder_get_right_speed_cm_s(void);
float encoder_get_left_rpm(void);
float encoder_get_right_rpm(void);
float encoder_get_left_distance_m(void);
float encoder_get_right_distance_m(void);

// Raw pulses
uint32_t encoder_get_left_pulses(void);
uint32_t encoder_get_right_pulses(void);

// Diagnostics (optional)
uint32_t encoder_get_left_noise_count(void);
uint32_t encoder_get_right_noise_count(void);
void encoder_reset_noise_counters(void);
void encoder_print_diagnostics(void);

#ifdef __cplusplus
}
#endif
#endif // ENCODER_H
