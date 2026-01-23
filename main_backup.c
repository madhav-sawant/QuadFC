/**
 * @file main.c
 * @brief Rate PID & Mixer Validation Firmware
 *
 * PURPOSE:
 * Validate the INNER (RATE) PID control loop and Motor Mixing.
 *
 * TEST PROCEDURE (NO PROPS!):
 * 1. Power on.
 * 2. System initializes (IMU, PWM, ADC).
 * 3. Gyro calibrates (Keep still).
 * 4. Safety Check: Battery voltage.
 * 5. ARMING SEQUENCE:
 *    - Wait 5 seconds.
 *    - Arm motors at IDLE speed.
 * 6. LOOP:
 *    - Read Gyro.
 *    - Run Rate PID (Target = 0 deg/s).
 *    - Mix outputs (Throttle = Idle).
 *    - Send to Motors.
 *    - Verify motors react to movement (correct direction).
 *
 * SAFETY:
 * - Low battery cutoff.
 * - Crash angle protection (disarm if > 60 deg tilt).
 * - Kill switch (Boot button GPIO 0).
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

#include "../lib/altitude/altitude_control.h"
#include "../lib/baro/baro.h"
#include "../lib/blackbox/blackbox.h"
#include "../lib/config/config.h"
#include "../lib/pwm/pwm.h"
#include "../lib/rx/rx.h"
#include "../lib/vl53l0x/vl53l0x.h"
#include "../lib/webserver/webserver.h"
#include "adc.h"
#include "imu.h"
#include "mixer.h"
#include "rate_control.h"

/* -------------------------------------------------------------------------- */
/*                               Configuration                                */
/* -------------------------------------------------------------------------- */

// Status LED
#define LED_PIN 2
#define BUTTON_PIN 0 // Boot button for emergency stop

// Throttle Limit for Tuning (prevents climbing too high)
// Set to 1400 during tuning, change to 2000 for normal flight
#define TUNING_THROTTLE_LIMIT 1400

// Control loop frequency (250 Hz)
#define CONTROL_LOOP_FREQ_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ_HZ)

// Debug print frequency (10 Hz) -> 250 / 25 = 10
#define DEBUG_PRINT_DIVIDER 25

// RC Mapping
#define RC_MAX_ANGLE_DEG 45.0f
#define RC_MAX_YAW_RATE_DPS 180.0f
#define RC_DEADBAND_US 50 // Increased from 20 - FlySky may not center exactly

/* -------------------------------------------------------------------------- */
/*                               Global State                                 */
/* -------------------------------------------------------------------------- */

static int debug_counter = 0;
static int blackbox_counter = 0;  // Counter for 83Hz blackbox logging
static int led_blink_counter = 0; // Counter for low battery LED blinking

// System State
bool system_armed = false;
static bool error_state = false;
char system_status_msg[64] = "System Ready"; // Global status message

// Debug data
// Debug data (Cleaned up: PID/Gyro used directly in print)
static uint16_t debug_motors[4];
static uint16_t debug_vbat = 0;
static int64_t debug_exec_time_us = 0;

// Sensor Fusion Globals (for Webserver visibility)
static float global_fused_alt_m = 0.0f;
static float global_baro_rel_m = 0.0f;
static float global_laser_m = 0.0f;

// Public Getters for Webserver
float get_fused_alt(void) { return global_fused_alt_m; }
float get_baro_alt(void) { return global_baro_rel_m; }
float get_laser_alt(void) { return global_laser_m; }
uint16_t get_battery_mv(void) { return debug_vbat; }

// Disarm reason tracking (for blackbox debugging)
// 0 = Not disarmed, 1 = Angle, 2 = Gyro Rate, 3 = RX Loss, 4 = E-Stop, 5 =
// Switch
static uint8_t last_disarm_reason = 0;

/* -------------------------------------------------------------------------- */
/*                   Control Loop Task (Core 1 - 250 Hz)                      */
/* -------------------------------------------------------------------------- */

// Control loop runs as a dedicated FreeRTOS task pinned to Core 1
// Core 0: WiFi, Webserver, Logger (background I/O)
// Core 1: Control Loop, Sensor Fusion, PWM (real-time flight critical)

static void control_loop_task(void *arg) {
  (void)arg;

  // Use esp_timer for precise 4ms (250Hz) timing since FreeRTOS tick is 10ms
  const int64_t cycle_period_us =
      1000000 / CONTROL_LOOP_FREQ_HZ; // 4000us for 250Hz
  int64_t next_cycle_time = esp_timer_get_time();

  // Angle Mode Integral State
  float angle_i_roll = 0.0f;
  float angle_i_pitch = 0.0f;
  const float MAX_ANGLE_I_OUTPUT =
      30.0f; // Limit I-term contribution to 30 deg/s

  printf("Control Loop Task started on Core %d\n", xPortGetCoreID());

  // Register this task with the Task Watchdog Timer (TWDT)
  esp_task_wdt_add(NULL);

  // Register this task with the Task Watchdog Timer (TWDT)
  esp_task_wdt_add(NULL);

  // Sensor Fusion State
  vl53l0x_t vl53_sensor;
  vl53l0x_init(&vl53_sensor); // Init struct (hw init done in main)
  float baro_offset_m = 0.0f;
  bool offset_locked = false;
  // fused_alt_m and baro_rel_m are now global
  const float FUSION_THRESH_M = 1.0f; // 1m Threshold (Use Laser when < 1m)
  float ground_alt_ref = 0.0f;
  int sensor_divider = 0;

  // Calibrate Baro Ground Ref at task start
  // (We assume IMU/I2C is already init)
  printf("FUSION: Calibrating Baro Ground Ref...\n");
  float start_alt_sum = 0.0f;
  for (int i = 0; i < 20; i++) {
    baro_read();
    start_alt_sum += baro_get_data()->altitude_m;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  ground_alt_ref = start_alt_sum / 20.0f;
  // Apply 6cm offset: baro sensor is 6cm higher than laser, subtract to align
  const float BARO_OFFSET_M = 0.06f;
  ground_alt_ref += BARO_OFFSET_M;
  printf("FUSION: Baro Ground Ref = %.2f m (with 6cm offset)\n",
         ground_alt_ref);

  // Calibrate Laser Ground Ref (10 samples)
  printf("FUSION: Calibrating Laser Ground Ref...\n");
  float laser_ground_ref_m = 0.0f;
  int laser_cal_count = 0;
  for (int i = 0; i < 10; i++) {
    uint16_t cal_mm = 0;
    esp_err_t ret = vl53l0x_read_single(&vl53_sensor, &cal_mm);
    if (ret == ESP_OK && cal_mm > 0 && cal_mm < 8190) {
      laser_ground_ref_m += (float)cal_mm / 1000.0f;
      laser_cal_count++;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  if (laser_cal_count > 0) {
    laser_ground_ref_m /= laser_cal_count;
  }
  printf("FUSION: Laser Ground Ref = %.2f m (from %d samples)\n",
         laser_ground_ref_m, laser_cal_count);

  while (1) {
    // Feed the watchdog at each iteration
    esp_task_wdt_reset();
    int64_t start_time = esp_timer_get_time();

    // 1. Read IMU
    imu_read(1.0f / CONTROL_LOOP_FREQ_HZ);
    const imu_data_t *imu = imu_get_data();

    // 1b. Sensor Fusion (25Hz - Every 10th cycle)
    if (++sensor_divider >= 10) {
      sensor_divider = 0;

      // Read Sensors
      uint16_t dist_mm = 0;
      // Use Single Shot mode (fixes MPU6050 conflict)
      esp_err_t ret = vl53l0x_read_single(&vl53_sensor, &dist_mm);
      bool laser_valid = (ret == ESP_OK) && (dist_mm > 0) && (dist_mm < 8190);
      // Apply ground reference offset (so altitude starts at 0)
      // Apply ground reference offset (so altitude starts at 0)
      global_laser_m = ((float)dist_mm / 1000.0f) - laser_ground_ref_m;

      // 1. Dynamic Grounding (Joop Brokking Method)
      // If we are on the ground (Laser < 5cm), slowly drift the ground
      // reference to match current baro reading. This kills the drift. Filter
      // coeff 0.01: very slow adaptation to avoid jumping during momentary
      // touchdowns
      if (laser_valid && global_laser_m < 0.05f) {
        ground_alt_ref =
            ground_alt_ref * 0.99f + baro_get_data()->altitude_m * 0.01f;
      }

      float baro_raw = baro_get_data()->altitude_m - ground_alt_ref;

      // 2. Baro Low-Pass Filter (Alpha = 0.05)
      static float baro_filtered = 0.0f;
      static bool baro_filter_init = false;

      if (!system_armed) {
        // Reset filter when disarmed (Clamp to 0)
        baro_filtered = 0.0f;
        baro_filter_init = true;
      } else if (!baro_filter_init) {
        baro_filtered = baro_raw; // Initialize on first armed reading
        baro_filter_init = true;
      } else {
        baro_filtered = 0.95f * baro_filtered + 0.05f * baro_raw;
      }
      global_baro_rel_m = baro_filtered;

      // FUSION LOGIC (Latching at 1m)
      if (laser_valid && global_laser_m < FUSION_THRESH_M) {
        // Below Threshold: Use Laser, Reset Lock
        global_fused_alt_m = global_laser_m;
        offset_locked = false;
      } else {
        // Above Threshold: Use Baro + Locked Offset
        if (!offset_locked) {
          // One-Shot Lock: Align Baro to Target Height (1m)
          baro_offset_m = global_baro_rel_m - FUSION_THRESH_M;
          offset_locked = true;
        }
        global_fused_alt_m = global_baro_rel_m - baro_offset_m;
      }
    }

    // 2. Safety Checks - Crash Detection
    // Angle-based: if roll or pitch angle is too high, disarm
    // (Angle Spike Filter in imu.c prevents vibration false triggers)
    const float CRASH_ANGLE_DEG = 60.0f;
    if (fabs(imu->roll_deg) > CRASH_ANGLE_DEG ||
        fabs(imu->pitch_deg) > CRASH_ANGLE_DEG) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      last_disarm_reason = 1; // Angle
      printf("DISARM: Angle Crash! R=%.1f P=%.1f\n", imu->roll_deg,
             imu->pitch_deg);
      snprintf(system_status_msg, sizeof(system_status_msg),
               "CRASH DETECTED: Angle > %.0f deg", CRASH_ANGLE_DEG);
    }

    // Gyro rate-based: detect rapid rotation (impact/crash)
    const float CRASH_GYRO_RATE_DPS = 2000.0f;
    if (fabs(imu->gyro_x_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_y_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_z_dps) > CRASH_GYRO_RATE_DPS) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      last_disarm_reason = 2; // Gyro Rate
      printf("DISARM: Gyro Crash! X=%.1f Y=%.1f Z=%.1f\n", imu->gyro_x_dps,
             imu->gyro_y_dps, imu->gyro_z_dps);
      snprintf(system_status_msg, sizeof(system_status_msg),
               "CRASH DETECTED: Gyro > %.0f dps", CRASH_GYRO_RATE_DPS);
    }

    // 3. Read RC Stick Inputs
    uint16_t rx_roll = rx_get_channel(0);
    uint16_t rx_pitch = rx_get_channel(1);
    uint16_t rx_thr = rx_get_channel(2);
    uint16_t rx_yaw = rx_get_channel(3);
    uint16_t rx_aux2 = rx_get_channel(5); // AUX2 for altitude hold mode switch
    // bool alt_hold_switch = (rx_aux2 > 1600); // High = altitude hold ON

    // ========================================================================
    // ANGLE MODE (Self-Leveling) - ALWAYS ACTIVE
    // ========================================================================
    // Control Flow:
    //   Stick -> Target Angle -> Angle Error -> Angle P -> Target Rate -> Rate
    //   PID
    // ========================================================================

    // 3a. Map Stick to Target Angle (±angle_max degrees)
    float target_roll_angle = 0.0f;
    if (abs(rx_roll - 1500) > RC_DEADBAND_US) {
      target_roll_angle = (float)(rx_roll - 1500) / 500.0f * sys_cfg.angle_max;
    }

    float target_pitch_angle = 0.0f;
    if (abs(rx_pitch - 1500) > RC_DEADBAND_US) {
      target_pitch_angle =
          (float)(rx_pitch - 1500) / 500.0f * sys_cfg.angle_max;
    }

    // 3b. Calculate Angle Error
    float roll_angle_error = target_roll_angle - imu->roll_deg;
    float pitch_angle_error = target_pitch_angle - imu->pitch_deg;

    // 3c. Angle PI Controller -> Outputs Target Rate (deg/s)
    // ------------------------------------------------------------------------
    // Integration (I-term) to fix steady-state error / drift
    if (!system_armed) {
      angle_i_roll = 0.0f;
      angle_i_pitch = 0.0f;
    }

    if (sys_cfg.angle_ki > 0.0f) {
      float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
      float max_integral = MAX_ANGLE_I_OUTPUT / sys_cfg.angle_ki;

      // Roll
      angle_i_roll += roll_angle_error * dt;
      if (angle_i_roll > max_integral)
        angle_i_roll = max_integral;
      else if (angle_i_roll < -max_integral)
        angle_i_roll = -max_integral;

      // Pitch
      angle_i_pitch += pitch_angle_error * dt;
      if (angle_i_pitch > max_integral)
        angle_i_pitch = max_integral;
      else if (angle_i_pitch < -max_integral)
        angle_i_pitch = -max_integral;
    } else {
      angle_i_roll = 0.0f;
      angle_i_pitch = 0.0f;
    }

    float target_roll_rate = (sys_cfg.angle_kp * roll_angle_error) +
                             (angle_i_roll * sys_cfg.angle_ki);
    float target_pitch_rate = (sys_cfg.angle_kp * pitch_angle_error) +
                              (angle_i_pitch * sys_cfg.angle_ki);

    // 3d. Clamp target rates to prevent excessive commands
    const float MAX_ANGLE_RATE = 150.0f; // Max rate output from angle loop
    if (target_roll_rate > MAX_ANGLE_RATE)
      target_roll_rate = MAX_ANGLE_RATE;
    if (target_roll_rate < -MAX_ANGLE_RATE)
      target_roll_rate = -MAX_ANGLE_RATE;
    if (target_pitch_rate > MAX_ANGLE_RATE)
      target_pitch_rate = MAX_ANGLE_RATE;
    if (target_pitch_rate < -MAX_ANGLE_RATE)
      target_pitch_rate = -MAX_ANGLE_RATE;

    // 3e. Yaw remains Rate Mode (no angle control for yaw)
    float target_yaw_rate = 0.0f;
    if (abs(rx_yaw - 1500) > RC_DEADBAND_US) {
      target_yaw_rate = (float)(rx_yaw - 1500) / 500.0f * RC_MAX_YAW_RATE_DPS;
    }

    // 4. Update webserver live display
    webserver_set_angle_targets(target_roll_angle, target_pitch_angle);
    webserver_set_rate_targets(target_roll_rate, target_pitch_rate);

    // 5. Rate PID Control (Inner Loop) - Takes rate setpoints from Angle Loop
    rate_control_update(target_roll_rate, target_pitch_rate, target_yaw_rate,
                        imu->gyro_x_dps, imu->gyro_y_dps, imu->gyro_z_dps);

    const rate_output_t *pid_out = rate_control_get_output();

    // 5. Mixer - Throttle Control with Optional Altitude Hold
    uint16_t throttle = system_armed ? rx_thr : 1000;

    // Safety: Ensure throttle doesn't drop below min or exceed max
    if (throttle < 1000)
      throttle = 1000;
    if (throttle > TUNING_THROTTLE_LIMIT)
      throttle = TUNING_THROTTLE_LIMIT;

    // ========================================================================
    // ALTITUDE HOLD MODE **DISABLED FOR TESTING**
    // ========================================================================
    // Altitude hold is temporarily disabled to isolate angle mode control
    // testing without barometer/laser interference on throttle.
    // ========================================================================
    /*
    static bool alt_hold_was_enabled = false;
    static float last_alt_target = 0.0f;

    if (system_armed && alt_hold_switch && throttle > 1200) {
      // Enable altitude hold
      if (!altitude_control_is_enabled()) {
        // Just activated - set target to current altitude
        altitude_control_set_target(global_fused_alt_m);
        altitude_control_enable(true);
        last_alt_target = global_fused_alt_m;
      }

      // Allow pilot to adjust target with throttle stick
      // If stick is significantly above/below center, change target
      if (rx_thr > 1550) {
        // Stick up: increase target altitude
        last_alt_target += 0.002f; // 2mm per loop = 0.5m/s climb rate at 250Hz
        altitude_control_set_target(last_alt_target);
      } else if (rx_thr < 1450) {
        // Stick down: decrease target altitude
        last_alt_target -= 0.002f;
        if (last_alt_target < 0.1f)
          last_alt_target = 0.1f; // Min 10cm
        altitude_control_set_target(last_alt_target);
      }

      // Get throttle adjustment from altitude PID
      float alt_correction = altitude_control_update(
          global_fused_alt_m, 1.0f / CONTROL_LOOP_FREQ_HZ);

      // Apply correction to base hover throttle
      // Base hover throttle is what's needed to hover at constant altitude
      int16_t corrected_throttle =
          ALT_BASE_HOVER_THROTTLE + (int16_t)alt_correction;

      // Clamp to safe range
      if (corrected_throttle < 1100)
        corrected_throttle = 1100;
      if (corrected_throttle > TUNING_THROTTLE_LIMIT)
        corrected_throttle = TUNING_THROTTLE_LIMIT;

      throttle = (uint16_t)corrected_throttle;
      alt_hold_was_enabled = true;
    } else {
      // Altitude hold OFF - manual throttle control
      if (alt_hold_was_enabled) {
        altitude_control_enable(false);
        alt_hold_was_enabled = false;
      }
    }
    */
    // ALTITUDE HOLD DISABLED - Throttle is now 100% manual from stick input

    mixer_update(throttle, pid_out->roll, pid_out->pitch, pid_out->yaw);

    int64_t end_time = esp_timer_get_time();

    // 6. Store Debug Data (for serial print)
    if (++debug_counter >= DEBUG_PRINT_DIVIDER) {
      debug_counter = 0;

      mixer_get_outputs(&debug_motors[0], &debug_motors[1], &debug_motors[2],
                        &debug_motors[3]);

      debug_exec_time_us = end_time - start_time;

      // DEBUG: Print all sensor data
      printf(
          "ALT:%+5.0fcm | BARO:%+5.0fcm | R:%+5.1f P:%+5.1f | %s | %d.%02dV\n",
          global_fused_alt_m * 100.0f, global_baro_rel_m * 100.0f,
          imu->roll_deg, imu->pitch_deg, offset_locked ? "BARO" : "LASER",
          debug_vbat / 1000, (debug_vbat % 1000) / 10);
    }

    // 6. Blackbox Logging (50Hz - every 10th iteration) - ONLY WHEN ARMED
    if (system_armed && ++blackbox_counter >= BLACKBOX_LOG_DIVIDER) {
      blackbox_counter = 0;

      // Calculate expanded flags
      uint16_t bb_flags = 0;
      if (error_state)
        bb_flags |= BLACKBOX_FLAG_ERROR;
      // Add low battery flag
      if (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold)
        bb_flags |= BLACKBOX_FLAG_LOW_BAT;
      // Gyro saturation check
      if (fabs(imu->gyro_x_dps) > 400 || fabs(imu->gyro_y_dps) > 400)
        bb_flags |= BLACKBOX_FLAG_GYRO_SAT;
      // PID saturation check
      if (fabs(pid_out->roll) > sys_cfg.rate_output_limit * 0.95f)
        bb_flags |= BLACKBOX_FLAG_PID_SAT;

      blackbox_entry_t entry = {
          // Time & Status
          .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
          .flags = bb_flags,

          // Raw IMU Data
          .gyro_x = imu->gyro_x_dps,
          .gyro_y = imu->gyro_y_dps,
          .gyro_z = imu->gyro_z_dps,
          .accel_x = imu->accel_x_g,
          .accel_y = imu->accel_y_g,
          .accel_z = imu->accel_z_g,

          // Rate Loop - setpoints and errors
          .rate_setpoint_roll = target_roll_rate,
          .rate_setpoint_pitch = target_pitch_rate,
          .rate_error_roll = target_roll_rate - imu->gyro_x_dps,
          .rate_error_pitch = target_pitch_rate - imu->gyro_y_dps,
          .rate_i_term_roll = rate_control_get_i_roll(),
          .rate_i_term_pitch = rate_control_get_i_pitch(),
          .rate_i_term_yaw = rate_control_get_i_yaw(),

          // PID Outputs
          .pid_roll = pid_out->roll,
          .pid_pitch = pid_out->pitch,
          .pid_yaw = pid_out->yaw,

          // Motors
          .motor = {debug_motors[0], debug_motors[1], debug_motors[2],
                    debug_motors[3]},

          // RC Inputs
          .rc_throttle = throttle,
          .rc_roll = rx_roll,
          .rc_pitch = rx_pitch,

          // New Sensor Data
          .fused_alt = global_fused_alt_m,
          .baro_alt = global_baro_rel_m,
          .laser_alt = global_laser_m,
          .angle_setpoint_roll = target_roll_angle,
          .angle_setpoint_pitch = target_pitch_angle,

          // System Health
          .battery_mv = debug_vbat,
          .loop_time_us = (uint16_t)debug_exec_time_us,
          .cpu_temp = 0, // TODO: Add ESP32 temperature reading
          .pad = 0};
      blackbox_log(&entry);
    } else if (!system_armed) {
      blackbox_counter = 0; // Reset counter when disarmed
    }

    // Wait for next cycle - precise timing using esp_timer
    // Busy-wait until next cycle time (FreeRTOS tick is 10ms, too slow for
    // 250Hz)
    next_cycle_time += cycle_period_us;
    while (esp_timer_get_time() < next_cycle_time) {
      // Tight loop - yields CPU briefly to avoid complete starvation
      taskYIELD();
    }
  }
}

/* -------------------------------------------------------------------------- */
/*                                 Main Entry                                 */
/* -------------------------------------------------------------------------- */

void app_main(void) {
  // ========== CRITICAL: ESC BOOT FIX ==========
  // Configure motor GPIOs as OUTPUT LOW immediately to prevent
  // ESCs from seeing floating signals during ESP32 boot.
  // This happens BEFORE any other initialization!
  gpio_reset_pin(PWM_MOTOR_1_GPIO); // Motor 1
  gpio_reset_pin(PWM_MOTOR_2_GPIO); // Motor 2
  gpio_reset_pin(PWM_MOTOR_3_GPIO); // Motor 3
  gpio_reset_pin(PWM_MOTOR_4_GPIO); // Motor 4
  gpio_set_direction(PWM_MOTOR_1_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_2_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_3_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(PWM_MOTOR_4_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(PWM_MOTOR_1_GPIO, 0);
  gpio_set_level(PWM_MOTOR_2_GPIO, 0);
  gpio_set_level(PWM_MOTOR_3_GPIO, 0);
  gpio_set_level(PWM_MOTOR_4_GPIO, 0);

  // Now init PWM properly (will take over these pins)
  pwm_init();

  // Send stable IDLE signal to all motors
  for (int i = 0; i < 4; i++) {
    pwm_set_motor(i, 1000);
  }

  // ========== END ESC BOOT FIX ==========

  // 1. Init NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Load Configuration
  config_load_defaults();
  config_load_from_nvs(); // Load saved values from flash (if any)

  // 2. Init GPIO
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  gpio_reset_pin(BUTTON_PIN);
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

  // 3. Init Subsystems
  adc_init();

  rx_init();
  mixer_init();
  blackbox_init();  // RAM-based flight data logger
  webserver_init(); // WiFi AP + PID tuning web interface

  if (imu_init() != ESP_OK) {
    printf("IMU Init Failed! (Continuing for Fusion Demo)\n");
    // Removed blocking loop to allow testing Baro/Laser
  }

  baro_init();

  // 4. Gyro/Accel Calibration with LED Feedback
  // COMBINED WAIT: ESC Arming (need time) + Placement (need time)
  printf(
      "BOOT: PWM Init Done. Place drone flat! Calibration in 3 seconds...\n");
  gpio_set_level(LED_PIN, 1);      // LED ON
  vTaskDelay(pdMS_TO_TICKS(3000)); // 3s for ESCs to arm AND user to place drone
  gpio_set_level(LED_PIN, 0);      // LED OFF - Calibration starting

  // ALWAYS calibrate accelerometer at startup (adapts to any surface)
  printf("IMU: Calibrating Accelerometer... KEEP LEVEL!\n");
  vTaskDelay(pdMS_TO_TICKS(500)); // Give user time to level
  imu_calibrate_accel();
  printf("IMU: Accel calibration done.\n");

  // THEN: Calibrate Gyro on every boot (gyro bias drifts with temperature)
  printf("IMU: Calibrating Gyro... KEEP STILL!\n");
  imu_calibrate_gyro();
  printf("IMU: Gyro Calibrated.\n");
  // NOTE: Do NOT save to NVS here - it would overwrite accel with current
  // values Accel calibration is only saved when done via webserver

  // Step 3: SUCCESS SIGNAL (Pulse LED 3 times)
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // ON
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100)); // OFF
  }
  vTaskDelay(pdMS_TO_TICKS(500)); // Pause

  // printf("BOOT: System Ready.\n");

  // ========== LEVEL CHECK ==========
  // Take multiple IMU readings and check if quad is level
  // This helps verify trim values are correct before flying
  // printf("\n");
  // printf("========================================\n");
  // printf("       LEVEL CHECK - Keep quad level!\n");
  // printf("========================================\n");
  // printf("Taking 100 samples...\n");

  float roll_sum = 0.0f;
  float pitch_sum = 0.0f;
  const int LEVEL_CHECK_SAMPLES = 100;

  // Wait for IMU to stabilize
  vTaskDelay(pdMS_TO_TICKS(500));

  // Collect samples
  for (int i = 0; i < LEVEL_CHECK_SAMPLES; i++) {
    imu_read(0.004f); // 4ms dt
    const imu_data_t *imu = imu_get_data();
    roll_sum += imu->roll_deg;
    pitch_sum += imu->pitch_deg;
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms between samples
  }

  float avg_roll = roll_sum / LEVEL_CHECK_SAMPLES;
  float avg_pitch = pitch_sum / LEVEL_CHECK_SAMPLES;

  // printf("\n");
  // printf("LEVEL CHECK RESULTS:\n");
  // printf("----------------------------------------\n");
  // printf("  Average Roll:  %+.2f degrees\n", avg_roll);
  // printf("  Average Pitch: %+.2f degrees\n", avg_pitch);
  // printf("----------------------------------------\n");

  // Check if level (within ±1 degree is acceptable)
  bool roll_ok = fabsf(avg_roll) < 1.0f;
  bool pitch_ok = fabsf(avg_pitch) < 1.0f;

  if (roll_ok && pitch_ok) {
    printf("  STATUS: ✓ LEVEL OK!\n");
    printf("  Quad is level, ready to fly.\n");
  } else {
    printf("  STATUS: ✗ NOT LEVEL - Adjust trim!\n");
    printf("\n");
    printf("  RECOMMENDED TRIM ADJUSTMENTS:\n");

    if (!pitch_ok) {
      if (avg_pitch > 0) {
        printf("  → PITCH_TRIM_DEG: Decrease by %.1f\n", avg_pitch);
        printf("    (Current pitch reads +%.1f, reduce trim)\n", avg_pitch);
      } else {
        printf("  → PITCH_TRIM_DEG: Increase by %.1f\n", -avg_pitch);
        printf("    (Current pitch reads %.1f, increase trim)\n", avg_pitch);
      }
    }

    if (!roll_ok) {
      if (avg_roll > 0) {
        printf("  → ROLL_TRIM_DEG: Decrease by %.1f\n", avg_roll);
        printf("    (Current roll reads +%.1f, reduce trim)\n", avg_roll);
      } else {
        printf("  → ROLL_TRIM_DEG: Increase by %.1f\n", -avg_roll);
        printf("    (Current roll reads %.1f, increase trim)\n", avg_roll);
      }
    }

    printf("\n");
    printf("  Edit lib/imu/imu.c lines 57-58 and rebuild.\n");
  }

  printf("========================================\n\n");
  // ========== END LEVEL CHECK ==========

  // 5. Init Rate Control Only
  rate_control_init();
  altitude_control_init(); // Initialize altitude hold PID

  // 6. Start Control Loop on Core 1 (dedicated for flight-critical code)
  // Core 0: WiFi, Webserver, Logger (background I/O)
  // Core 1: Control Loop, Sensor Fusion, PWM (real-time flight critical)
  xTaskCreatePinnedToCore(control_loop_task, // Task function
                          "control_loop",    // Task name
                          4096,              // Stack size (bytes)
                          NULL,              // Task parameters
                          24,   // Priority (high - flight critical)
                          NULL, // Task handle (not needed)
                          1     // Core 1 (isolated from WiFi)
  );

  // printf("Control Loop Task created on Core 1.\n");

  // 8. Main Loop (Monitoring & Arming)
  while (1) {
    // 7. Arming & Safety Logic (RX Channel 5) - runs every iteration
    // Implement Debounce for EMI protection
    bool instant_rx_ok = rx_is_connected();
    static int rx_fail_counter = 0;

    if (instant_rx_ok) {
      rx_fail_counter = 0;
    } else {
      rx_fail_counter++;
    }

    // Only consider RX lost if missing for > 20 iterations (200ms)
    // This allows short EMI glitches to pass without disarming
    bool rx_ok = (rx_fail_counter < 20);

    if (!instant_rx_ok && rx_ok && (rx_fail_counter % 5 == 0)) {
      // printf("WARN: RX Signal Glitch (EMI?) - Ignored (%d/20)\n",
      //        rx_fail_counter);
    }

    uint16_t rx_aux1 = rx_get_channel(4);    // Channel 5 (0-indexed is 4)
    bool arm_switch_high = (rx_aux1 > 1600); // Threshold for arming

    // Arming State Machine
    if (rx_ok && arm_switch_high && !error_state) {
      if (!system_armed) {
        // Check throttle safety before arming
        uint16_t rx_thr_check = rx_get_channel(2);

        // DISABLED: Battery check removed for testing
        // uint16_t bat_check = adc_read_battery_voltg();
        // bool battery_ok = (bat_check > 9900);

        if (rx_thr_check >= 1150) {
          static int warn_counter = 0;
          if (warn_counter++ % 50 == 0) {
            // printf("CANNOT ARM: Throttle not low!\n");
          }
        } else {
          // All checks passed - ARM!
          system_armed = true;

          // Reset PIDs to prevent I-term windup
          rate_control_init();

          mixer_arm(true);
          blackbox_clear(); // Clear old data
          blackbox_start(); // Start recording new flight
          // printf("ARMED! (Switch High)\n");
        }
      }
    } else {
      if (system_armed) {
        system_armed = false;
        mixer_arm(false);
        blackbox_stop(); // Stop recording, preserve data for download
        if (!rx_ok) {
          last_disarm_reason = 3; // RX Loss
          // printf("DISARM: RX Signal Lost! Failsafe Triggered\n");
        } else {
          last_disarm_reason = 5; // Switch Low
          // printf("DISARM: Switch Low\n");
        }
      }

      // Clear error state if disarmed and switch is low (allow re-arming)
      if (error_state && !arm_switch_high) {
        error_state = false;
        // printf("Error State Cleared. Ready to Arm.\n");
      }
    }

    // Low Battery Check (Slow)
    static int val_bat_divider = 0;
    if (++val_bat_divider >= 100) { // 1Hz
      val_bat_divider = 0;
      debug_vbat = adc_read_battery_voltg();
    }
    bool low_battery_warning =
        (debug_vbat < sys_cfg.low_bat_threshold); // Below 10.5V

    // LED Status: Blink if low battery, otherwise solid when armed
    if (low_battery_warning) {
      // Blink LED as warning (toggle every ~25 iterations = ~250ms at 10ms
      // loop)
      led_blink_counter++;
      if (led_blink_counter >= 25) {
        led_blink_counter = 0;
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN)); // Toggle LED
      }
    } else {
      led_blink_counter = 0;
      gpio_set_level(LED_PIN, system_armed ? 1 : 0);
    }

    // Check Emergency Stop Button (GPIO 0 / Boot Button)
    if (gpio_get_level(BUTTON_PIN) == 0) {
      system_armed = false;
      mixer_arm(false);
      last_disarm_reason = 4; // Emergency Stop
      // printf("DISARM: Emergency Stop Button!\n");
      error_state = true;
      snprintf(system_status_msg, sizeof(system_status_msg),
               "EMERGENCY STOP: Button Pressed");
    }

    // DISABLED: Battery auto-disarm removed for testing
    // WARNING: Monitor battery voltage manually!
    // static int crit_bat_counter = 0;
    // if (system_armed && debug_vbat > 5000 &&
    //     debug_vbat <= 9000) { // Critical <= 9.0V
    //   crit_bat_counter++;
    //   if (crit_bat_counter > 50) {
    //     system_armed = false;
    //     mixer_arm(false);
    //     ...
    //   }
    // }

    vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz Main Loop
  }
}
