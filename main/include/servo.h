#ifndef SERVO_H
#define SERVO_H

#include "driver/ledc.h"
#include "esp_err.h"

// Servo parameters
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180

// LEDC parameters
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_GPIO CONFIG_SERVO_OUTPUT_GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_12_BIT

/**
 * Initialize the servo motor using LEDC
 */
void servo_init(void);

/**
 * Convert an angle to the corresponding duty cycle
 * @param angle Angle in degrees (0-180)
 * @return Calculated duty cycle value
 */
uint32_t servo_angle_to_duty(int angle);

/**
 * Set the servo to a specific angle
 * @param angle Angle in degrees (0-180)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_set_angle(int angle);

#endif // SERVO_H
