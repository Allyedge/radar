#include "radar.h"
#include "sensor.h"
#include "servo.h"
#include <stdio.h>
#include <string.h>

static bool angles_alert[MAX_ANGLES] = {false};
static int alert_count = 0;

void radar_init(void) {
  memset(angles_alert, 0, sizeof(angles_alert));
  alert_count = 0;
}

static void handle_normal_sweep(int *angle, int *direction, sweep_mode_t *mode);
static void process_alert_clusters(bool angles_alert[], bool current_alerts[],
                                   int alert_positions[],
                                   int *num_alert_positions);
static bool check_and_update_alert(int alert_angle, bool current_alerts[],
                                   int alert_positions[],
                                   int *num_alert_positions,
                                   int *current_alert_index);

void servo_sweep_task(void *pvParameter) {
  int angle = 0;
  int direction = 1;
  sweep_mode_t mode = SWEEP_NORMAL;
  bool current_alerts[MAX_ANGLES] = {false};
  int alert_positions[MAX_ANGLES] = {0};
  int num_alert_positions = 0;
  int current_alert_index = 0;

  alert_state_t alert_state = ALERT_STATE_NEXT;
  int64_t last_state_change = 0;

  while (1) {
    if (mode == SWEEP_NORMAL) {
      handle_normal_sweep(&angle, &direction, &mode);

      if (mode == SWEEP_ALERT) {
        process_alert_clusters(angles_alert, current_alerts, alert_positions,
                               &num_alert_positions);
        current_alert_index = 0;
        alert_state = ALERT_STATE_NEXT;
      }
    } else {
      if (num_alert_positions == 0) {
        mode = SWEEP_NORMAL;
        printf("SWITCH: NORMAL mode - all alerts cleared\n");
        memset(angles_alert, 0, sizeof(angles_alert));
        alert_count = 0;
        continue;
      }

      switch (alert_state) {
      case ALERT_STATE_NEXT:
        if (current_alert_index >= num_alert_positions) {
          current_alert_index = 0;
        }

        int alert_angle = alert_positions[current_alert_index];
        if (alert_angle < 0 || alert_angle >= MAX_ANGLES) {
          printf("ERROR: Invalid alert angle: %d\n", alert_angle);
          current_alert_index = (current_alert_index + 1) % num_alert_positions;
          continue;
        }

        printf("MOVING: to alert angle %d\n", alert_angle);
        esp_err_t result = servo_set_angle(alert_angle);
        if (result != ESP_OK) {
          printf("ERROR: Failed to set servo angle: %d\n", alert_angle);
          current_alert_index = (current_alert_index + 1) % num_alert_positions;
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }

        alert_state = ALERT_STATE_MOVING;
        last_state_change = esp_timer_get_time();
        break;

      case ALERT_STATE_MOVING:
        if ((esp_timer_get_time() - last_state_change) >= 300000) {
          alert_state = ALERT_STATE_MEASURING;
          last_state_change = esp_timer_get_time();
          printf("REACHED: position for angle %d, now measuring\n",
                 alert_positions[current_alert_index]);
        }
        break;

      case ALERT_STATE_MEASURING:
        alert_angle = alert_positions[current_alert_index];

        if (check_and_update_alert(alert_angle, current_alerts, alert_positions,
                                   &num_alert_positions,
                                   &current_alert_index)) {
          alert_state = ALERT_STATE_NEXT;
        } else {
          if (num_alert_positions > 1) {
            current_alert_index =
                (current_alert_index + 1) % num_alert_positions;
          }
          vTaskDelay(pdMS_TO_TICKS(200));
          alert_state = ALERT_STATE_NEXT;
        }
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

static void handle_normal_sweep(int *angle, int *direction,
                                sweep_mode_t *mode) {
  esp_err_t result = servo_set_angle(*angle);
  if (result != ESP_OK) {
    printf("ERROR: Failed to set servo angle: %d\n", *angle);
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(15));
  float distance_cm = measure_distance_cm();
  printf("SWEEP: angle: %d, distance: %.2f cm\n", *angle, distance_cm);

  if (distance_cm <= ALERT_DISTANCE_CM) {
    printf("ALERT: angle: %d, distance: %.2f cm\n", *angle, distance_cm);
    angles_alert[*angle] = true;
    alert_count++;
  }

  *angle += STEP_SIZE * (*direction);
  if (*angle >= SERVO_MAX_DEGREE || *angle <= SERVO_MIN_DEGREE) {
    *direction = -(*direction);

    if (alert_count > 0) {
      *mode = SWEEP_ALERT;
      printf("SWITCH: ALERT mode with %d raw alerts\n", alert_count);
    }
  }
}

static void process_alert_clusters(bool angles_alert[], bool current_alerts[],
                                   int alert_positions[],
                                   int *num_alert_positions) {
  bool in_cluster = false;
  int cluster_start = 0;
  int cluster_end = 0;

  *num_alert_positions = 0;
  memset(current_alerts, 0, sizeof(bool) * MAX_ANGLES);

  for (int i = 0; i <= SERVO_MAX_DEGREE; i++) {
    if (angles_alert[i]) {
      if (!in_cluster) {
        in_cluster = true;
        cluster_start = i;
      }
      cluster_end = i;
    } else if (in_cluster) {
      int center_angle = (cluster_start + cluster_end) / 2;
      if (*num_alert_positions < MAX_ANGLES) {
        alert_positions[(*num_alert_positions)++] = center_angle;
        current_alerts[center_angle] = true;
      }
      in_cluster = false;
    }
  }

  if (in_cluster && *num_alert_positions < MAX_ANGLES) {
    int center_angle = (cluster_start + cluster_end) / 2;
    alert_positions[(*num_alert_positions)++] = center_angle;
    current_alerts[center_angle] = true;
  }

  printf("CLUSTER: Reduced to %d distinct objects\n", *num_alert_positions);
}

static bool check_and_update_alert(int alert_angle, bool current_alerts[],
                                   int alert_positions[],
                                   int *num_alert_positions,
                                   int *current_alert_index) {
  float distance_cm = measure_distance_cm();
  printf("CHECK ALERT: angle: %d, distance: %.2f cm\n", alert_angle,
         distance_cm);

  if (distance_cm > ALERT_DISTANCE_CM) {
    printf("CLEARED: angle: %d, distance: %.2f cm\n", alert_angle, distance_cm);

    for (int i = alert_angle - ANGLE_CLUSTER_TOLERANCE;
         i <= alert_angle + ANGLE_CLUSTER_TOLERANCE; i++) {
      if (i >= 0 && i < MAX_ANGLES) {
        if (angles_alert[i]) {
          angles_alert[i] = false;
          alert_count--;
        }
      }
    }

    current_alerts[alert_angle] = false;

    *num_alert_positions = 0;
    for (int i = 0; i < MAX_ANGLES; i++) {
      if (current_alerts[i]) {
        alert_positions[(*num_alert_positions)++] = i;
      }
    }

    if (*num_alert_positions > 0) {
      *current_alert_index = *current_alert_index % *num_alert_positions;
    }

    return true;
  }

  return false;
}
