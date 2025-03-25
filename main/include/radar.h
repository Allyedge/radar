#ifndef RADAR_H
#define RADAR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// Radar parameters
#define MAX_ANGLES 181
#define ALERT_DISTANCE_CM CONFIG_ALERT_DISTANCE_CM
#define ANGLE_CLUSTER_TOLERANCE CONFIG_ANGLE_CLUSTER_TOLERANCE
#define STEP_SIZE CONFIG_STEP_SIZE

// Sweep modes
typedef enum { SWEEP_NORMAL, SWEEP_ALERT } sweep_mode_t;

// Alert states
typedef enum {
  ALERT_STATE_MOVING,
  ALERT_STATE_MEASURING,
  ALERT_STATE_NEXT
} alert_state_t;

/**
 * Initialize the radar system
 */
void radar_init(void);

/**
 * Task function for servo sweep operation
 * @param pvParameter Task parameters (unused)
 */
void servo_sweep_task(void *pvParameter);

#endif // RADAR_H
