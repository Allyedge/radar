#include "sensor.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include <stdio.h>

static volatile int64_t echo_start_time = 0;
static volatile int64_t echo_end_time = 0;
static volatile bool echo_received = false;
static SemaphoreHandle_t echo_semaphore = NULL;

static void IRAM_ATTR echo_isr_handler(void *arg) {
  int pin_state = gpio_get_level(SENSOR_ECHO_GPIO);

  if (pin_state == 1) {
    echo_start_time = esp_timer_get_time();
  } else {
    echo_end_time = esp_timer_get_time();
    echo_received = true;
    xSemaphoreGiveFromISR(echo_semaphore, NULL);
  }
}

void sensor_init(void) {
  gpio_config_t trigger_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = 1ULL << SENSOR_TRIGGER_GPIO,
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&trigger_conf);

  gpio_config_t echo_conf = {
      .intr_type = GPIO_INTR_ANYEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = 1ULL << SENSOR_ECHO_GPIO,
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&echo_conf);

  echo_semaphore = xSemaphoreCreateBinary();
  gpio_install_isr_service(0);
  gpio_isr_handler_add(SENSOR_ECHO_GPIO, echo_isr_handler, NULL);
}

float measure_distance_cm(void) {
  echo_received = false;

  gpio_set_level(SENSOR_TRIGGER_GPIO, 0);
  vTaskDelay(pdMS_TO_TICKS(2));
  gpio_set_level(SENSOR_TRIGGER_GPIO, 1);
  ets_delay_us(10);
  gpio_set_level(SENSOR_TRIGGER_GPIO, 0);

  if (xSemaphoreTake(echo_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (echo_received && echo_end_time > echo_start_time) {
      int64_t duration_us = echo_end_time - echo_start_time;
      float distance_cm = duration_us * 0.034 / 2;
      return distance_cm;
    }
  }

  return 400.0;
}
