#ifndef SENSOR_H
#define SENSOR_H

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

// Ultrasonic sensor pins
#define SENSOR_TRIGGER_GPIO CONFIG_SENSOR_TRIGGER_GPIO
#define SENSOR_ECHO_GPIO CONFIG_SENSOR_ECHO_GPIO

/**
 * Initialize the ultrasonic sensor
 */
void sensor_init(void);

/**
 * Measure distance using the ultrasonic sensor
 * @return Distance in centimeters, 400.0 if measurement failed
 */
float measure_distance_cm(void);

#endif // SENSOR_H
