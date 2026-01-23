/**
 * @file main.c
 * @brief Quadcopter Flight Controller - ESP32
 *
 * Cascaded control architecture:
 * - Outer Loop: Angle PI (self-leveling)
 * - Inner Loop: Rate PID (stabilization)
 *
 * Safety features: Crash detection, RX failsafe, emergency stop
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "adc.h"
#include "blackbox.h"
#include "config.h"
#include "imu.h"
#include "mixer.h"
#include "pwm.h"
#include "rate_control.h"
#include "angle_control.h"
#include "rx.h"
#include "webserver.h"

/* ─────────────────────────────────────────────────────────────────────────────
 * Configuration
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define LED_PIN 2
#define BUTTON_PIN 0 // Boot button = emergency stop

#define CONTROL_LOOP_FREQ_HZ 250
#define TUNING_THROTTLE_LIMIT 1400 // Safety limit during tuning

#define RC_DEADBAND_US 50
#define RC_MAX_YAW_RATE_DPS 180.0f

#define CRASH_ANGLE_DEG 60.0f
#define CRASH_GYRO_RATE_DPS 2000.0f

/* ─────────────────────────────────────────────────────────────────────────────
 * Global State
 * ─────────────────────────────────────────────────────────────────────────────
 */
bool system_armed = false;
static bool error_state = false;
char system_status_msg[64] = "Ready";

static uint16_t debug_motors[4];
static uint16_t debug_vbat = 0;
static int64_t debug_exec_time_us = 0;

// Webserver API
float get_fused_alt(void) { return 0.0f; }
float get_baro_alt(void) { return 0.0f; }
float get_laser_alt(void) { return 0.0f; }
uint16_t get_battery_mv(void) { return debug_vbat; }

/* ─────────────────────────────────────────────────────────────────────────────
 * Control Loop Task (Core 1, 250Hz)
 * ─────────────────────────────────────────────────────────────────────────────
 */
static void control_loop_task(void *arg) {
  (void)arg;

  const int64_t cycle_us = 1000000 / CONTROL_LOOP_FREQ_HZ;
  int64_t next_cycle = esp_timer_get_time();

  // Angle Mode (now handled by angle_control)
  // const float MAX_ANGLE_RATE = 150.0f; // Moved to angle_control.c

  int debug_div = 0;
  int bb_div = 0;

  printf("Control loop started (Core %d)\n", xPortGetCoreID());
  esp_task_wdt_add(NULL);

  while (1) {
    esp_task_wdt_reset();
    int64_t start = esp_timer_get_time();

    // 1. Read IMU
    imu_read(1.0f / CONTROL_LOOP_FREQ_HZ);
    const imu_data_t *imu = imu_get_data();

    // 2. Crash detection
    if (fabs(imu->roll_deg) > CRASH_ANGLE_DEG ||
        fabs(imu->pitch_deg) > CRASH_ANGLE_DEG) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      webserver_set_error("CRASH: Angle exceeded!");
      printf("CRASH: Angle exceeded\n");
    }
    if (fabs(imu->gyro_x_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_y_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_z_dps) > CRASH_GYRO_RATE_DPS) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      webserver_set_error("CRASH: Gyro rate exceeded!");
      printf("CRASH: Gyro rate exceeded\n");
    }

    // 3. Read RC inputs
    uint16_t rx_roll = rx_get_channel(0);
    uint16_t rx_pitch = rx_get_channel(1);
    uint16_t rx_thr = rx_get_channel(2);
    uint16_t rx_yaw = rx_get_channel(3);

    // 4. Angle Mode (outer loop)
    float target_roll_angle = 0.0f;
    float target_pitch_angle = 0.0f;

    if (abs(rx_roll - 1500) > RC_DEADBAND_US) {
      target_roll_angle = (float)(rx_roll - 1500) / 500.0f * sys_cfg.angle_max;
    }
    if (abs(rx_pitch - 1500) > RC_DEADBAND_US) {
      target_pitch_angle =
          (float)(rx_pitch - 1500) / 500.0f * sys_cfg.angle_max;
    }

    // Update Angle Controller
    angle_control_update(target_roll_angle, target_pitch_angle, 
                         imu->roll_deg, imu->pitch_deg, 
                         1.0f / CONTROL_LOOP_FREQ_HZ, 
                         system_armed, rx_thr);
    
    // Get Angle Loop Output (Target Rates)
    const angle_output_t *angle_out = angle_control_get_output();
    float target_roll_rate = angle_out->target_roll_rate;
    float target_pitch_rate = angle_out->target_pitch_rate;

    // Yaw: rate mode only
    float target_yaw_rate = 0.0f;
    if (abs(rx_yaw - 1500) > RC_DEADBAND_US) {
      target_yaw_rate = (float)(rx_yaw - 1500) / 500.0f * RC_MAX_YAW_RATE_DPS;
    }

    // Update webserver display
    webserver_set_angle_targets(target_roll_angle, target_pitch_angle);
    webserver_set_rate_targets(target_roll_rate, target_pitch_rate);

    // 5. Rate PID (inner loop)
    rate_control_update(target_roll_rate, target_pitch_rate, target_yaw_rate,
                        imu->gyro_x_dps, imu->gyro_y_dps, imu->gyro_z_dps);
    const rate_output_t *pid = rate_control_get_output();

    // 6. Throttle and mixer
    uint16_t throttle = system_armed ? rx_thr : 1000;
    if (throttle < 1000)
      throttle = 1000;
    if (throttle > TUNING_THROTTLE_LIMIT)
      throttle = TUNING_THROTTLE_LIMIT;

    // Freeze I-term at low throttle
    rate_control_freeze_integral(!system_armed || throttle < 1150);

    mixer_update(throttle, pid->roll, pid->pitch, pid->yaw);

    int64_t end = esp_timer_get_time();

    // Debug output (10Hz)
    if (++debug_div >= 25) {
      debug_div = 0;
      mixer_get_outputs(&debug_motors[0], &debug_motors[1], &debug_motors[2],
                        &debug_motors[3]);
      debug_exec_time_us = end - start;
    }

    // Blackbox logging (50Hz, armed only)
    if (system_armed && ++bb_div >= BLACKBOX_LOG_DIVIDER) {
      bb_div = 0;

      uint16_t flags = 0;
      if (error_state)
        flags |= BLACKBOX_FLAG_ERROR;
      if (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold)
        flags |= BLACKBOX_FLAG_LOW_BAT;
      if (fabs(imu->gyro_x_dps) > 400 || fabs(imu->gyro_y_dps) > 400)
        flags |= BLACKBOX_FLAG_GYRO_SAT;
      if (fabs(pid->roll) > sys_cfg.rate_output_limit * 0.95f)
        flags |= BLACKBOX_FLAG_PID_SAT;

      blackbox_entry_t entry = {
          .angle_roll = imu->roll_deg,
          .angle_pitch = imu->pitch_deg,
          .gyro_x = imu->gyro_x_dps,
          .gyro_y = imu->gyro_y_dps,
          .gyro_z = imu->gyro_z_dps,
          .pid_roll = pid->roll,
          .pid_pitch = pid->pitch,
          .pid_yaw = pid->yaw,
          .motor = {debug_motors[0], debug_motors[1], debug_motors[2],
                    debug_motors[3]},
          .rc_roll = rx_roll,
          .rc_pitch = rx_pitch,
          .rc_yaw = rx_yaw,
          .rc_throttle = throttle,
          .battery_mv = debug_vbat,
          .i2c_errors = imu_get_i2c_errors(),
          .accel_z_raw = imu->accel_z_g,
      };
      blackbox_log(&entry);
    } else if (!system_armed) {
      bb_div = 0;
    }

    // Wait for next cycle
    next_cycle += cycle_us;
    while (esp_timer_get_time() < next_cycle) {
      taskYIELD();
    }
  }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Main Entry
 * ─────────────────────────────────────────────────────────────────────────────
 */
void app_main(void) {
  // ESC boot fix: hold motor pins LOW during init
  gpio_reset_pin(PWM_MOTOR_1_GPIO);
  gpio_reset_pin(PWM_MOTOR_2_GPIO);
  gpio_reset_pin(PWM_MOTOR_3_GPIO);
  gpio_reset_pin(PWM_MOTOR_4_GPIO);
  gpio_set_direction(PWM_MOTOR_1_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_2_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_3_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_4_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(PWM_MOTOR_1_GPIO, 0);
  gpio_set_level(PWM_MOTOR_2_GPIO, 0);
  gpio_set_level(PWM_MOTOR_3_GPIO, 0);
  gpio_set_level(PWM_MOTOR_4_GPIO, 0);

  pwm_init();
  for (int i = 0; i < 4; i++) {
    pwm_set_motor(i, 1000);
  }

  // NVS init
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  // Load config
  config_load_defaults();
  config_load_from_nvs();

  // GPIO setup
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_reset_pin(BUTTON_PIN);
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

  // Initialize subsystems
  adc_init();
  rx_init();
  mixer_init();
  blackbox_init();
  webserver_init();

  // Initialize IMU with error detection
  if (imu_init() != ESP_OK) {
    webserver_set_error("SENSOR ERROR: IMU (MPU6050) not detected!");
    error_state = true;
    snprintf(system_status_msg, sizeof(system_status_msg), "IMU FAIL");
  }

  // Calibration sequence
  printf("Place drone flat, calibrating in 3s...\n");
  gpio_set_level(LED_PIN, 1);
  vTaskDelay(pdMS_TO_TICKS(3000));
  gpio_set_level(LED_PIN, 0);

  printf("Calibrating accelerometer...\n");
  vTaskDelay(pdMS_TO_TICKS(500));
  imu_calibrate_accel();

  printf("Calibrating gyro...\n");
  imu_calibrate_gyro();

  // Success indication
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Level check
  vTaskDelay(pdMS_TO_TICKS(500));
  float roll_sum = 0, pitch_sum = 0;
  for (int i = 0; i < 100; i++) {
    imu_read(0.004f);
    const imu_data_t *imu = imu_get_data();
    roll_sum += imu->roll_deg;
    pitch_sum += imu->pitch_deg;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  float avg_roll = roll_sum / 100.0f;
  float avg_pitch = pitch_sum / 100.0f;

  if (fabsf(avg_roll) < 1.0f && fabsf(avg_pitch) < 1.0f) {
    printf("Level check: OK\n");
  } else {
    printf("Level check: FAILED (R=%.1f, P=%.1f)\n", avg_roll, avg_pitch);
  }

  // Start control loop
  rate_control_init();
  angle_control_init();
  xTaskCreatePinnedToCore(control_loop_task, "control", 4096, NULL, 24, NULL,
                          1);

  // Main loop: arming and safety
  static int rx_fail = 0;
  static int bat_div = 0;
  static int led_blink = 0;

  while (1) {
    // RX failsafe with debounce
    bool rx_instant = rx_is_connected();
    rx_fail = rx_instant ? 0 : rx_fail + 1;
    bool rx_ok = (rx_fail < 20);

    // RX failsafe error reporting
    if (!rx_ok) {
      webserver_set_error("RX FAILSAFE: No receiver signal!");
    }

    uint16_t rx_aux1 = rx_get_channel(4);
    bool arm_sw = (rx_aux1 > 1600);

    // Arming logic
    if (rx_ok && arm_sw && !error_state) {
      if (!system_armed && rx_get_channel(2) < 1150) {
        system_armed = true;
        rate_control_init();
        angle_control_init();
        mixer_arm(true);
        blackbox_clear();
        blackbox_start();
      }
    } else {
      if (system_armed) {
        system_armed = false;
        mixer_arm(false);
        blackbox_stop();
      }
      if (error_state && !arm_sw) {
        error_state = false;
        webserver_set_error(""); // Clear error on reset
      }
    }

    // Battery monitoring (1Hz)
    if (++bat_div >= 100) {
      bat_div = 0;
      debug_vbat = adc_read_battery_voltg();

      // Low battery warning
      if (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold) {
        webserver_set_error("LOW BATTERY!");
      }
    }

    // LED status
    bool low_bat = (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold);
    if (low_bat) {
      if (++led_blink >= 25) {
        led_blink = 0;
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
      }
    } else {
      led_blink = 0;
      gpio_set_level(LED_PIN, system_armed ? 1 : 0);
    }

    // Emergency stop button
    if (gpio_get_level(BUTTON_PIN) == 0) {
      system_armed = false;
      mixer_arm(false);
      error_state = true;
      webserver_set_error("EMERGENCY STOP!");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
