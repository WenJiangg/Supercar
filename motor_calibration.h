/**
 * ==============================================================================
 * MOTOR CALIBRATION SYSTEM - For Perfect Demo Day Performance
 * ==============================================================================
 */

#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdlib.h>

// ==== CALIBRATION CONSTANTS ====
// Updated with noise-filtered encoder measurements
#define LEFT_MOTOR_CALIBRATION   1.000f
#define RIGHT_MOTOR_CALIBRATION  0.987f

// Minimum and maximum speed limits
#define MOTOR_MIN_SPEED  0
#define MOTOR_MAX_SPEED  100

// ==== MOTOR PIN DEFINITIONS ====
#define MOTOR_LEFT_A        8
#define MOTOR_LEFT_B        9
#define MOTOR_RIGHT_A       10
#define MOTOR_RIGHT_B       11

// *** FIX: Changed from 12500 (10kHz) to 4999 (25kHz) to stop whine/brownouts ***
#define PWM_WRAP            4999

// ==============================================================================
// INTERNAL MOTOR CONTROL
// ==============================================================================
static inline void _set_motor_raw(uint pin_a, uint pin_b, int speed, int direction) {
    uint slice_a = pwm_gpio_to_slice_num(pin_a);
    uint slice_b = pwm_gpio_to_slice_num(pin_b);
    uint chan_a = pwm_gpio_to_channel(pin_a);
    uint chan_b = pwm_gpio_to_channel(pin_b);
    
    if (speed < MOTOR_MIN_SPEED) speed = MOTOR_MIN_SPEED;
    if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    
    uint16_t duty = (PWM_WRAP * speed) / 100;
    
    if (direction == 1) {
        pwm_set_chan_level(slice_a, chan_a, duty);
        pwm_set_chan_level(slice_b, chan_b, 0);
    } else if (direction == -1) {
        pwm_set_chan_level(slice_a, chan_a, 0);
        pwm_set_chan_level(slice_b, chan_b, duty);
    } else {
        pwm_set_chan_level(slice_a, chan_a, 0);
        pwm_set_chan_level(slice_b, chan_b, 0);
    }
}

// ==============================================================================
// CALIBRATED MOTOR CONTROL
// ==============================================================================

static inline void motor_forward(int speed) {
    int left_speed = (int)(speed * LEFT_MOTOR_CALIBRATION);
    int right_speed = (int)(speed * RIGHT_MOTOR_CALIBRATION);
    
    _set_motor_raw(MOTOR_LEFT_A, MOTOR_LEFT_B, left_speed, 1);
    _set_motor_raw(MOTOR_RIGHT_A, MOTOR_RIGHT_B, right_speed, 1);
}

static inline void motor_backward(int speed) {
    int left_speed = (int)(speed * LEFT_MOTOR_CALIBRATION);
    int right_speed = (int)(speed * RIGHT_MOTOR_CALIBRATION);
    
    _set_motor_raw(MOTOR_LEFT_A, MOTOR_LEFT_B, left_speed, -1);
    _set_motor_raw(MOTOR_RIGHT_A, MOTOR_RIGHT_B, right_speed, -1);
}

static inline void motor_turn_right(int speed, float turn_ratio) {
    int left_speed = (int)(speed * LEFT_MOTOR_CALIBRATION);
    int right_speed = (int)(speed * RIGHT_MOTOR_CALIBRATION * turn_ratio);
    
    _set_motor_raw(MOTOR_LEFT_A, MOTOR_LEFT_B, left_speed, 1);
    _set_motor_raw(MOTOR_RIGHT_A, MOTOR_RIGHT_B, right_speed, 1);
}

static inline void motor_turn_left(int speed, float turn_ratio) {
    int left_speed = (int)(speed * LEFT_MOTOR_CALIBRATION * turn_ratio);
    int right_speed = (int)(speed * RIGHT_MOTOR_CALIBRATION);
    
    _set_motor_raw(MOTOR_LEFT_A, MOTOR_LEFT_B, left_speed, 1);
    _set_motor_raw(MOTOR_RIGHT_A, MOTOR_RIGHT_B, right_speed, 1);
}

static inline void motor_rotate_right(int speed) {
    int left_speed = (int)(speed * LEFT_MOTOR_CALIBRATION);
    int right_speed = (int)(speed * RIGHT_MOTOR_CALIBRATION);
    
    _set_motor_raw(MOTOR_LEFT_A, MOTOR_LEFT_B, left_speed, 1);
    _set_motor_raw(MOTOR_RIGHT_A, MOTOR_RIGHT_B, right_speed, -1);
}

static inline void motor_rotate_left(int speed) {
    int left_speed = (int)(speed * LEFT_MOTOR_CALIBRATION);
    int right_speed = (int)(speed * RIGHT_MOTOR_CALIBRATION);
    
    _set_motor_raw(MOTOR_LEFT_A, MOTOR_LEFT_B, left_speed, -1);
    _set_motor_raw(MOTOR_RIGHT_A, MOTOR_RIGHT_B, right_speed, 1);
}

static inline void motor_stop(void) {
    _set_motor_raw(MOTOR_LEFT_A, MOTOR_LEFT_B, 0, 0);
    _set_motor_raw(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0, 0);
}

static inline void motor_set_differential(int left_speed, int right_speed) {
    int left_cal = (int)(abs(left_speed) * LEFT_MOTOR_CALIBRATION);
    int right_cal = (int)(abs(right_speed) * RIGHT_MOTOR_CALIBRATION);
    
    int left_dir = (left_speed > 0) ? 1 : (left_speed < 0) ? -1 : 0;
    int right_dir = (right_speed > 0) ? 1 : (right_speed < 0) ? -1 : 0;
    
    _set_motor_raw(MOTOR_LEFT_A, MOTOR_LEFT_B, left_cal, left_dir);
    _set_motor_raw(MOTOR_RIGHT_A, MOTOR_RIGHT_B, right_cal, right_dir);
}

static inline void motor_init(void) {
    gpio_set_function(MOTOR_LEFT_A, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_LEFT_B, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_A, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_B, GPIO_FUNC_PWM);
    
    uint slice_la = pwm_gpio_to_slice_num(MOTOR_LEFT_A);
    uint slice_lb = pwm_gpio_to_slice_num(MOTOR_LEFT_B);
    uint slice_ra = pwm_gpio_to_slice_num(MOTOR_RIGHT_A);
    uint slice_rb = pwm_gpio_to_slice_num(MOTOR_RIGHT_B);
    
    // Set clock divider to 1.0f
    pwm_set_clkdiv(slice_la, 1.0f);
    pwm_set_clkdiv(slice_lb, 1.0f);
    pwm_set_clkdiv(slice_ra, 1.0f);
    pwm_set_clkdiv(slice_rb, 1.0f);

    pwm_set_wrap(slice_la, PWM_WRAP);
    pwm_set_wrap(slice_lb, PWM_WRAP);
    pwm_set_wrap(slice_ra, PWM_WRAP);
    pwm_set_wrap(slice_rb, PWM_WRAP);
    
    pwm_set_enabled(slice_la, true);
    pwm_set_enabled(slice_lb, true);
    pwm_set_enabled(slice_ra, true);
    pwm_set_enabled(slice_rb, true);
    
    motor_stop();
}

// Demo presets
// NOTE: These are the original blocking versions.
// The new non-blocking ones are in robot_controller.c
static inline void motor_demo_turn_90_right(void) {
    motor_rotate_right(40);
    sleep_ms(650);
    motor_stop();
}

static inline void motor_demo_turn_90_left(void) {
    motor_rotate_left(40);
    sleep_ms(650);
    motor_stop();
}

static inline void motor_demo_uturn(void) {
    motor_rotate_right(40);
    sleep_ms(1300);
    motor_stop();
}

#endif // MOTOR_CALIBRATION_H