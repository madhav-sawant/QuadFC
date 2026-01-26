/**
 * @file main.c
 * @brief Quadcopter Flight Controller - Final Year Project
 * @author 4th Year Engineering Student
 * 
 * This is the main control loop for my quadcopter project.
 * Uses cascaded PID control - outer loop for angle, inner loop for rate.
 * Spent way too many nights debugging this thing.
 * 
 * Control Architecture:
 * - Outer Loop: Angle PI controller (maintains level flight)
 * - Inner Loop: Rate PID controller (handles fast corrections)
 * 
 * Hardware: ESP32 + MPU6050 + F450 frame + 1400KV motors + 8045 props
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
#include "blackbox.h"  // Flight data logging
#include "config.h"
#include "imu.h"
#include "mixer.h"
#include "pwm.h"
#include "rate_control.h"
#include "angle_control.h"
#include "rx.h"
#include "webserver.h"

/* =============================================================================
 * CONFIGURATION PARAMETERS
 * These values were tuned through extensive flight testing (and many crashes)
 * =============================================================================
 */
#define LED_PIN 2           // Onboard LED - useful for status indication
#define BUTTON_PIN 0        // Boot button - used for calibration and emergency stop

#define CONTROL_LOOP_FREQ_HZ 250    // 250Hz = 4ms loop time, fast enough for stable flight
#define TUNING_THROTTLE_LIMIT 2000  // MAX POWER: Full 2000us authority for recovery

#define RC_DEADBAND_US 50           // Stick deadband to prevent jitter
#define RC_MAX_YAW_RATE_DPS 180.0f  // Max yaw rotation speed in degrees per second

// Safety thresholds - if exceeded, something has gone very wrong
#define CRASH_ANGLE_DEG 90.0f       // Increased to 90 to allow recovery from steep angles (wind/TPA)
#define CRASH_GYRO_RATE_DPS 2000.0f // Spinning this fast = definitely crashing

/* =============================================================================
 * GLOBAL STATE VARIABLES
 * Kept minimal for safety - don't want race conditions mid-flight
 * =============================================================================
 */
bool system_armed = false;      // True when motors are live - BE CAREFUL!
static bool error_state = false; // Latched error flag - requires pilot to reset

// Auto-Centering Variables
// Captures stick position when armed to treat as "center"
static uint16_t rc_center_roll = 1500;
static uint16_t rc_center_pitch = 1500;

static uint16_t debug_motors[4]; // Current motor PWM values for logging
static uint16_t debug_vbat = 0;  // Battery voltage in millivolts

/* =============================================================================
 * CONTROL LOOP TASK
 * This is where the magic happens - runs at 250Hz on CPU core 1
 * Implements the cascaded PID control structure from control theory class
 * =============================================================================
 */
static void control_loop_task(void *arg) {
  (void)arg; // Unused parameter, but required by FreeRTOS

  // Calculate cycle time in microseconds for precise timing
  const int64_t cycle_us = 1000000 / CONTROL_LOOP_FREQ_HZ;
  int64_t next_cycle = esp_timer_get_time();

  int debug_div = 0;  // Divider for debug output (we don't need 250Hz debugging)
  int bb_div = 0;     // Divider for blackbox logging

  printf("Control loop started on Core %d - let's fly!\n", xPortGetCoreID());
  esp_task_wdt_add(NULL); // Add this task to watchdog - resets if we hang

  while (1) {
    esp_task_wdt_reset(); // Pet the watchdog, we're still alive

    // -------------------------------------------------------------------------
    // STEP 1: Read IMU data
    // The MPU6050 gives us gyro rates and accelerometer data
    // Complementary filter fuses them into stable angle estimates
    // -------------------------------------------------------------------------
    imu_read(1.0f / CONTROL_LOOP_FREQ_HZ);
    const imu_data_t *imu = imu_get_data();

    // -------------------------------------------------------------------------
    // STEP 2: Crash detection
    // If the drone is tumbling or spinning out of control, kill the motors
    // Better to crash softly than to have a runaway quadcopter
    // -------------------------------------------------------------------------
    if (fabs(imu->roll_deg) > CRASH_ANGLE_DEG ||
        fabs(imu->pitch_deg) > CRASH_ANGLE_DEG) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      // printf removed - causes delay in crash path
    }
    if (fabs(imu->gyro_x_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_y_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_z_dps) > CRASH_GYRO_RATE_DPS) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      // printf removed - causes delay in crash path
    }

    // -------------------------------------------------------------------------
    // STEP 3: Read RC receiver inputs
    // IBUS protocol gives us 1000-2000us PWM values for each channel
    // Channel mapping: 0=Roll, 1=Pitch, 2=Throttle, 3=Yaw, 4=Arm switch
    // -------------------------------------------------------------------------
    uint16_t rx_roll = rx_get_channel(0);
    uint16_t rx_pitch = rx_get_channel(1);
    uint16_t rx_thr = rx_get_channel(2);
    uint16_t rx_yaw = rx_get_channel(3);

    // -------------------------------------------------------------------------
    // STEP 4: OUTER LOOP - Angle Controller
    // Converts pilot's stick input to desired tilt angle
    // Then calculates rate command to achieve that angle
    // This is the self-leveling part - returns to level when sticks released
    // -------------------------------------------------------------------------
    float target_roll_angle = 0.0f;
    float target_pitch_angle = 0.0f;

    // Apply RC trim - uses captured center from arming moment
    // This allows zero drift even if TX/RX center is off (e.g. 1520 vs 1500)
    int16_t roll_trim = rx_roll - rc_center_roll;
    int16_t pitch_trim = rx_pitch - rc_center_pitch;
    
    // Apply deadband
    if (abs(roll_trim) > RC_DEADBAND_US) {
      target_roll_angle = (float)(roll_trim) / 500.0f * sys_cfg.angle_max;
    }
    if (abs(pitch_trim) > RC_DEADBAND_US) {
      target_pitch_angle = (float)(pitch_trim) / 500.0f * sys_cfg.angle_max;
    }

    // Run the angle PI controller
    // Output is target rotation rate (degrees per second)
    angle_control_update(target_roll_angle, target_pitch_angle, 
                         imu->roll_deg, imu->pitch_deg, 
                         1.0f / CONTROL_LOOP_FREQ_HZ, 
                         system_armed, rx_thr);
    
    const angle_output_t *angle_out = angle_control_get_output();
    float target_roll_rate = angle_out->target_roll_rate;
    float target_pitch_rate = angle_out->target_pitch_rate;

    // Yaw doesn't have angle hold - just rate control
    // (Heading hold would need a magnetometer)
    float target_yaw_rate = 0.0f;
    // Increased deadband to 80us specifically for Yaw to prevent accidental input during throttle
    if (abs(rx_yaw - 1500) > 80) {
      target_yaw_rate = (float)(rx_yaw - 1500) / 500.0f * RC_MAX_YAW_RATE_DPS;
    }

    // -------------------------------------------------------------------------
    // STEP 5: INNER LOOP - Rate PID Controller
    // This is the fast loop that actually stabilizes the drone
    // Compares target rate vs actual gyro rate and outputs motor commands
    // -------------------------------------------------------------------------
    rate_control_update(target_roll_rate, target_pitch_rate, target_yaw_rate,
                        imu->gyro_x_dps, imu->gyro_y_dps, imu->gyro_z_dps);
    const rate_output_t *pid = rate_control_get_output();

    // -------------------------------------------------------------------------
    // STEP 6: Motor Mixing
    // Takes throttle + roll/pitch/yaw corrections and calculates individual
    // motor speeds using the X-configuration mixing matrix
    // -------------------------------------------------------------------------
    uint16_t throttle = system_armed ? rx_thr : 1000;
    
    // Clamp throttle for safety
    if (throttle < 1000) throttle = 1000;
    if (throttle > TUNING_THROTTLE_LIMIT) throttle = TUNING_THROTTLE_LIMIT;

    // Freeze integral terms when not flying to prevent windup
    // This was a painful lesson learned from many flips on takeoff
    rate_control_freeze_integral(!system_armed || throttle < 1150);

    mixer_update(throttle, pid->roll, pid->pitch, pid->yaw);

    // -------------------------------------------------------------------------
    // Debug output at 10Hz (every 25 cycles)
    // Don't want to spam the console but need some visibility
    // -------------------------------------------------------------------------
    if (++debug_div >= 25) {
      debug_div = 0;
      mixer_get_outputs(&debug_motors[0], &debug_motors[1], 
                        &debug_motors[2], &debug_motors[3]);
    }

    // -------------------------------------------------------------------------
    // Blackbox logging (ENABLED)
    // Records flight data for post-flight analysis at ~83Hz
    // -------------------------------------------------------------------------
    if (system_armed && ++bb_div >= BLACKBOX_LOG_DIVIDER) {
      bb_div = 0;
      blackbox_entry_t entry = {
          .angle_roll = imu->roll_deg,
          .angle_pitch = imu->pitch_deg,
          .gyro_x = imu->gyro_x_dps,
          .gyro_y = imu->gyro_y_dps,
          .gyro_z = imu->gyro_z_dps,
          .pid_roll = pid->roll,
          .pid_pitch = pid->pitch,
          .pid_yaw = pid->yaw,
          .motor = {debug_motors[0], debug_motors[1], 
                    debug_motors[2], debug_motors[3]},
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

    // -------------------------------------------------------------------------
    // Wait for next cycle - precise timing is crucial for PID stability
    // Using busy-wait with yield for best timing accuracy
    // -------------------------------------------------------------------------
    next_cycle += cycle_us;
    while (esp_timer_get_time() < next_cycle) {
      taskYIELD(); // Let other tasks run while we wait
    }
  }
}

/* =============================================================================
 * IMU CALIBRATION FUNCTION
 * This is critical - bad calibration = bad flight
 * Must be done on a perfectly level surface!
 * =============================================================================
 */
static void perform_calibration(bool save_to_nvs) {
  // LED: SOLID ON = calibration in progress
  gpio_set_level(LED_PIN, 1);
  
  // Wait 2 seconds for user to step away from drone
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  // Accelerometer calibration
  imu_calibrate_accel();
  
  // Gyroscope calibration
  imu_calibrate_gyro();
  
  // Save to flash
  if (save_to_nvs) {
    imu_calibration_save_to_nvs();
  }
  
  gpio_set_level(LED_PIN, 0);
  
  // LED: 5 FAST BLINKS = calibration complete and saved!
  for (int i = 0; i < 5; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* =============================================================================
 * MAIN ENTRY POINT
 * This is where execution starts. Initializes everything and starts the
 * control loop. Took forever to get the startup sequence right.
 * =============================================================================
 */
void app_main(void) {
  // -------------------------------------------------------------------------
  // ESC Boot Fix
  // Some ESCs beep or arm if they see floating pins during boot
  // Hold motor pins LOW until we're ready to send proper PWM
  // Learned this the hard way when my quad tried to take off on the bench
  // -------------------------------------------------------------------------
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

  // Now initialize proper PWM and set all motors to minimum throttle
  pwm_init();
  for (int i = 0; i < 4; i++) {
    pwm_set_motor(i, 1000); // 1000us = motor off
  }

  // -------------------------------------------------------------------------
  // NVS (Non-Volatile Storage) Initialization
  // This is where we store PID gains and calibration data
  // Survives power cycles - super useful for saving tuned values
  // -------------------------------------------------------------------------
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // Flash is corrupted or new - erase and reinitialize
    printf("NVS: Erasing flash and reinitializing...\n");
    nvs_flash_erase();
    nvs_flash_init();
  }

  // Load configuration (PID gains, limits, etc.)
  config_load_defaults(); // Start with known good values
  config_load_from_nvs(); // Override with saved values if they exist

  // -------------------------------------------------------------------------
  // GPIO Setup
  // LED for status, button for calibration/emergency stop
  // -------------------------------------------------------------------------
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_reset_pin(BUTTON_PIN);
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

  // -------------------------------------------------------------------------
  // Initialize all the subsystems
  // Order matters here - some depend on others
  // -------------------------------------------------------------------------
  adc_init();       // Battery voltage monitoring
  rx_init();        // RC receiver (IBUS protocol)
  mixer_init();     // Motor mixing
  blackbox_init();  // Flight data logging
  
  // Initialize Webserver (WiFi AP)
  // Connect to 'QuadFC_Setup' 
  webserver_init();

  // -------------------------------------------------------------------------
  // IMU Initialization
  // This is the heart of the system - if IMU fails, we can't fly
  // -------------------------------------------------------------------------
  if (imu_init() != ESP_OK) {
    printf("\n!!! CRITICAL ERROR !!!\n");
    printf("IMU (MPU6050) not detected on I2C bus!\n");
    printf("Check your wiring: SDA=21, SCL=22\n");
    error_state = true;
    
    // Infinite blink loop - can't continue without IMU
    while(1) {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(100));
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  // -------------------------------------------------------------------------
  // Startup Banner
  // Makes it look professional in the serial monitor :)
  // -------------------------------------------------------------------------
  // LED: 3 SLOW BLINKS = "Hold BOOT button now to calibrate"
  // -------------------------------------------------------------------------
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  
  bool button_pressed = (gpio_get_level(BUTTON_PIN) == 0);
  
  if (button_pressed) {
    // CALIBRATION MODE - button held during slow blinks
    // Wait for button release first
    while (gpio_get_level(BUTTON_PIN) == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    
    perform_calibration(true);  // Calibrate and save to NVS
    
  } else {
    // NORMAL BOOT - load calibration from NVS
    bool accel_cal_loaded = imu_calibration_load_from_nvs();
    
    if (!accel_cal_loaded) {
      // No saved calibration - do temporary calibration
      // LED: SOLID ON 2 sec = temp calibration (not saved!)
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(2000));
      gpio_set_level(LED_PIN, 0);
      imu_calibrate_accel();
    }
    
    // Gyro always calibrates at boot
    imu_calibrate_gyro();
  }

  // LED: 3 QUICK BLINKS = boot successful
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Level Check - verify calibration looks reasonable
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

  if (fabsf(avg_roll) < 2.0f && fabsf(avg_pitch) < 2.0f) {
    // LED: 2 QUICK BLINKS = level OK
    for(int i=0; i<2; i++) {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(100));
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  } else {
    // LED: 5 SLOW BLINKS = level warning (not level or needs recalibration)
    for(int i=0; i<5; i++) {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(300));
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }

  // Start the control loop on Core 1

  rate_control_init();
  angle_control_init();
  xTaskCreatePinnedToCore(control_loop_task, "control", 4096, NULL, 24, NULL, 1);

  // -------------------------------------------------------------------------
  // Main loop - handles arming, safety, and housekeeping
  // Runs at ~100Hz on Core 0
  // -------------------------------------------------------------------------
  static int rx_fail = 0;   // Counter for RX failsafe
  static int bat_div = 0;   // Divider for battery check (1Hz)

  while (1) {
    // Check if receiver is connected (timeout-based)
    bool rx_instant = rx_is_connected();
    rx_fail = rx_instant ? 0 : rx_fail + 1;
    bool rx_ok = (rx_fail < 20); // ~200ms timeout

    // RX failsafe handled silently - printf removed for timing

    // Check arm switch (usually channel 5)
    uint16_t rx_aux1 = rx_get_channel(4);
    bool arm_sw = (rx_aux1 > 1600);

    // -----------------------------------------------------------------------
    // Arming Logic
    // To arm: RX connected + arm switch on + no errors + throttle low
    // The throttle check prevents accidental arming at high throttle
    // -----------------------------------------------------------------------
    if (rx_ok && arm_sw && !error_state) {
      if (!system_armed && rx_get_channel(2) < 1150) {
        system_armed = true;
        rate_control_init();   // Reset PID controllers
        angle_control_init();
        
        // AUTO-LEVEL ON ARM: Capture current angle as "level"
        // This allows drone to take off from ANY surface without drift!
        imu_set_level_on_arm();
        
        // AUTO-CENTER: Capture current stick positions as new center
        rc_center_roll = rx_get_channel(0);  // Roll
        rc_center_pitch = rx_get_channel(1); // Pitch
        
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
      // Clear error state when arm switch is turned off
      if (error_state && !arm_sw) {
        error_state = false;
        // printf removed for timing
      }
    }

    // -----------------------------------------------------------------------
    // Battery Monitoring (1Hz)
    // LiPo batteries don't like being over-discharged
    // -----------------------------------------------------------------------
    if (++bat_div >= 100) {
      bat_div = 0;
      debug_vbat = adc_read_battery_voltg();

      if (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold) {
        // Low battery warning - printf removed for timing
      }
    }

    // -----------------------------------------------------------------------
    // LED Status Indication
    // - Solid ON = low battery (also disarms!)
    // - Solid ON = armed
    // - Off = disarmed
    // -----------------------------------------------------------------------
    bool low_bat = (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold);
    if (low_bat) {
      // Low battery - DISARM and turn LED solid ON
      if (system_armed) {
        system_armed = false;
        mixer_arm(false);
        // printf removed for timing
      }
      gpio_set_level(LED_PIN, 1);  // Solid ON for low battery
    } else {
      gpio_set_level(LED_PIN, system_armed ? 1 : 0);
    }

    // -----------------------------------------------------------------------
    // Emergency Stop
    // If boot button is pressed while armed, immediately kill motors
    // This is the "oh crap" button
    // -----------------------------------------------------------------------
    if (system_armed && gpio_get_level(BUTTON_PIN) == 0) {
      system_armed = false;
      mixer_arm(false);
      error_state = true;
      // printf removed for timing
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // ~100Hz loop rate
  }
}
