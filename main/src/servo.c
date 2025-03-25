#include "servo.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

void servo_init(void) {
  ledc_timer_config_t ledc_timer = {.duty_resolution = LEDC_DUTY_RESOLUTION,
                                    .freq_hz = 50,
                                    .speed_mode = LEDC_MODE,
                                    .timer_num = LEDC_TIMER,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {.channel = LEDC_CHANNEL,
                                        .duty = 0,
                                        .gpio_num = LEDC_OUTPUT_GPIO,
                                        .speed_mode = LEDC_MODE,
                                        .timer_sel = LEDC_TIMER,
                                        .hpoint = 0};
  ledc_channel_config(&ledc_channel);
}

uint32_t servo_angle_to_duty(int angle) {
  if (angle < 0) {
    angle = 0;
  } else if (angle > SERVO_MAX_DEGREE) {
    angle = SERVO_MAX_DEGREE;
  }

  uint32_t pulsewidth_us =
      SERVO_MIN_PULSEWIDTH_US +
      ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) /
          SERVO_MAX_DEGREE;

  uint32_t duty = (pulsewidth_us * ((1 << LEDC_DUTY_RESOLUTION) - 1)) / 20000;
  return duty;
}

esp_err_t servo_set_angle(int angle) {
  uint32_t duty = servo_angle_to_duty(angle);
  esp_err_t err = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
  if (err == ESP_OK) {
    err = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
  }
  return err;
}
