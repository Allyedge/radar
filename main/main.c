#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "radar.h"
#include "sensor.h"
#include "servo.h"

void app_main(void) {
  sensor_init();
  servo_init();
  radar_init();

  xTaskCreate(servo_sweep_task, "servo_sweep_task", 4096, NULL, 5, NULL);
}
