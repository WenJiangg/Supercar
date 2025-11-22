#include "servo_control.h"
#include "hardware/gpio.h" // Needed for gpio_set_function
#include <stdio.h> // For printf

void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config cfg = pwm_get_default_config();
    // Configure for 50Hz (20ms period) with 1MHz clock
    // Clock div = 125 (125MHz / 125 = 1MHz)
    // Wrap = 20000 (1MHz / 20000 = 50Hz)
    pwm_config_set_clkdiv(&cfg, 125.0f);
    pwm_config_set_wrap(&cfg, 20000);
    pwm_init(slice, &cfg, true); // Start PWM
    printf("Servo PWM initialized on GP%d.\n", SERVO_PIN);
}

void servo_set_angle_instant(float deg) {
    // Clamp angle to the servo's physical limits
    if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
    if (deg > SERVO_MAX_DEG) deg = SERVO_MAX_DEG;

    // Calculate the pulse width in microseconds
    // Proportion = (angle - min_angle) / (max_angle - min_angle)
    float proportion = (deg - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
    // Pulse width = min_pulse + proportion * (max_pulse - min_pulse)
    uint16_t pulse_us = (uint16_t)(SERVO_MIN_US + proportion * (SERVO_MAX_US - SERVO_MIN_US));

    // Set the PWM duty cycle (channel level)
    // Level = pulse_us (since clock is 1MHz, 1 count = 1us)
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint chan = pwm_gpio_to_channel(SERVO_PIN);
    pwm_set_chan_level(slice, chan, pulse_us);
}
