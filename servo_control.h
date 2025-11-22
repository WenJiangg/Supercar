#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// ===== SERVO CONFIGURATION =====
// Make these accessible if needed by main logic, otherwise keep internal
#define SERVO_PIN           12
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_MIN_DEG       0.0f
#define SERVO_MAX_DEG       180.0f

/**
 * @brief Initializes PWM for the servo motor.
 */
void servo_init(void);

/**
 * @brief Sets the servo position based on an angle in degrees (Instantaneous).
 * Clamps the angle to the defined min/max servo range.
 * @param deg Target angle (0 to 180).
 */
void servo_set_angle_instant(float deg);

#endif // SERVO_CONTROL_H
