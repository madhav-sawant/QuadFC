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
// #include "blackbox.h"  // Disabled - uncomment to enable flight logging
#include "config.h"
#include "imu.h"
#include "mixer.h"
#include "pwm.h"
#include "rate_control.h"
#include "angle_control.h"
#include "rx.h"

/* =============================================================================
 * CONFIGURATION PARAMETERS
 * These values were tuned through extensive flight testing (and many crashes)
 * =============================================================================
 */
#define LED_PIN 2           // Onboard LED - useful for status indication
#define BUTTON_PIN 0        // Boot button - used for calibration and emergency stop

#define CONTROL_LOOP_FREQ_HZ 250    // 250Hz = 4ms loop time, fast enough for stable flight
#define TUNING_THROTTLE_LIMIT 1650  // Safety limit - don't want it flying away during testing

#define RC_DEADBAND_US 30           // Stick deadband to prevent jitter
#define RC_MAX_YAW_RATE_DPS 180.0f  // Max yaw rotation speed in degrees per second

// Safety thresholds - if exceeded, something has gone very wrong
#define CRASH_ANGLE_DEG 60.0f       // Drone shouldn't tilt more than 60 degrees
#define CRASH_GYRO_RATE_DPS 2000.0f // Spinning this fast = definitely crashing

/* =============================================================================
 * GLOBAL STATE VARIABLES
 * Kept minimal for safety - don't want race conditions mid-flight
 * =============================================================================
 */
bool system_armed = false;      // True when motors are live - BE CAREFUL!
static bool error_state = false; // Latched error flag - requires pilot to reset

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
  // int bb_div = 0;     // Divider for blackbox logging (disabled)

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
      printf("CRASH DETECTED: Angle exceeded %.0f degrees!\n", CRASH_ANGLE_DEG);
    }
    if (fabs(imu->gyro_x_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_y_dps) > CRASH_GYRO_RATE_DPS ||
        fabs(imu->gyro_z_dps) > CRASH_GYRO_RATE_DPS) {
      mixer_arm(false);
      system_armed = false;
      error_state = true;
      printf("CRASH DETECTED: Gyro rate exceeded %.0f dps!\n", CRASH_GYRO_RATE_DPS);
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

    // Apply deadband and convert stick position to angle
    // Stick center = 1500us, deflection ±500us maps to ±angle_max degrees
    if (abs(rx_roll - 1500) > RC_DEADBAND_US) {
      target_roll_angle = (float)(rx_roll - 1500) / 500.0f * sys_cfg.angle_max;
    }
    if (abs(rx_pitch - 1500) > RC_DEADBAND_US) {
      target_pitch_angle = (float)(rx_pitch - 1500) / 500.0f * sys_cfg.angle_max;
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
    if (abs(rx_yaw - 1500) > RC_DEADBAND_US) {
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
    // Blackbox logging (DISABLED - uncomment to enable)
    // Records flight data for post-flight analysis
    // -------------------------------------------------------------------------
    /*
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
    */

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
  printf("\n========================================\n");
  printf("       IMU CALIBRATION SEQUENCE\n");
  printf("========================================\n");
  printf("IMPORTANT: Keep the drone FLAT and STILL!\n");
  printf("Don't touch it during calibration.\n\n");
  
  // LED on = calibration in progress
  gpio_set_level(LED_PIN, 1);
  vTaskDelay(pdMS_TO_TICKS(2000)); // Give user time to step away
  
  // Accelerometer calibration - finds the "level" reference
  // This measures gravity vector and calculates offset angles
  printf("Step 1/2: Calibrating accelerometer...\n");
  imu_calibrate_accel();
  
  // Gyroscope calibration - finds the zero-rate bias
  // Gyros have a small offset that drifts with temperature
  printf("Step 2/2: Calibrating gyroscope...\n");
  imu_calibrate_gyro();
  
  // Optionally save to flash memory for persistence
  if (save_to_nvs) {
    printf("\nSaving calibration to flash memory...\n");
    imu_calibration_save_to_nvs();
    printf("SUCCESS: Calibration saved permanently!\n");
    printf("You won't need to recalibrate on next boot.\n");
  } else {
    printf("\nCalibration NOT saved (temporary only)\n");
  }
  
  // Show what we got
  imu_print_calibration();
  
  // Success indication - 3 quick blinks
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  gpio_set_level(LED_PIN, 0);
  
  printf("\nCalibration complete! Ready to fly.\n");
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
  // blackbox_init();  // Flight data logging (disabled)

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
  printf("\n");
  printf("╔═══════════════════════════════════════════════════════╗\n");
  printf("║       QUADCOPTER FLIGHT CONTROLLER v1.0               ║\n");
  printf("║       Final Year Engineering Project                  ║\n");
  printf("╠═══════════════════════════════════════════════════════╣\n");
  printf("║  Hardware: ESP32 + MPU6050 + F450 + 1400KV + 8045     ║\n");
  printf("║  Control:  Cascaded PID @ 250Hz                       ║\n");
  printf("╠═══════════════════════════════════════════════════════╣\n");
  printf("║  >>> Hold BOOT button now to calibrate IMU <<<        ║\n");
  printf("╚═══════════════════════════════════════════════════════╝\n\n");
  
  // Give user 1 second to press the button
  gpio_set_level(LED_PIN, 1);
  vTaskDelay(pdMS_TO_TICKS(1000));
  gpio_set_level(LED_PIN, 0);
  
  bool button_pressed = (gpio_get_level(BUTTON_PIN) == 0);
  
  if (button_pressed) {
    // -----------------------------------------------------------------------
    // CALIBRATION MODE
    // User held the button - do full calibration and save to NVS
    // Must be done on a perfectly level surface!
    // -----------------------------------------------------------------------
    printf("\n*** CALIBRATION MODE ACTIVATED ***\n");
    printf("Place drone on a LEVEL surface and don't touch it!\n\n");
    
    // Wait for button release so we don't start calibrating while they're
    // still messing with the button
    while (gpio_get_level(BUTTON_PIN) == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Let things settle
    
    perform_calibration(true);  // true = save to NVS
    
  } else {
    // -----------------------------------------------------------------------
    // NORMAL BOOT
    // Try to load calibration from NVS, fall back to temp cal if not found
    // -----------------------------------------------------------------------
    printf("Normal boot - loading calibration...\n\n");
    
    bool accel_cal_loaded = imu_calibration_load_from_nvs();
    
    if (accel_cal_loaded) {
      printf("✓ Accelerometer calibration loaded from flash\n");
    } else {
      printf("✗ No saved calibration found\n");
      printf("  Tip: Hold BOOT button at startup to calibrate properly\n\n");
      printf("  Doing temporary calibration for this session...\n");
      
      // Temporary calibration - not saved, user should calibrate properly
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(2000));
      gpio_set_level(LED_PIN, 0);
      imu_calibrate_accel();
    }
    
    // Gyro always needs calibration at boot - it drifts with temperature
    printf("Calibrating gyroscope...\n");
    imu_calibrate_gyro();
    
    imu_print_calibration();
  }

  // Success blinks - 3 quick ones
  for (int i = 0; i < 3; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // -------------------------------------------------------------------------
  // Level Check
  // Sanity check to make sure the calibration looks reasonable
  // If roll/pitch aren't near zero, something is wrong
  // -------------------------------------------------------------------------
  printf("\nPerforming level check...\n");
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
    printf("✓ Level check PASSED (Roll=%.1f°, Pitch=%.1f°)\n", avg_roll, avg_pitch);
    // Two quick blinks for success
    for(int i=0; i<2; i++) {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(100));
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  } else {
    printf("⚠ Level check WARNING (Roll=%.1f°, Pitch=%.1f°)\n", avg_roll, avg_pitch);
    printf("  Drone may not be level or needs recalibration\n");
    // Slow warning blinks - but continue anyway
    for(int i=0; i<5; i++) {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(300));
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }

  // -------------------------------------------------------------------------
  // Start the control loop
  // This runs on Core 1 at priority 24 (highest)
  // Core 0 handles WiFi and other system tasks (but we disabled WiFi)
  // -------------------------------------------------------------------------
  printf("\n*** FLIGHT CONTROLLER READY ***\n");
  printf("Arm switch: Channel 5 > 1600\n");
  printf("Throttle must be low to arm\n\n");
  
  rate_control_init();
  angle_control_init();
  xTaskCreatePinnedToCore(control_loop_task, "control", 4096, NULL, 24, NULL, 1);

  // -------------------------------------------------------------------------
  // Main loop - handles arming, safety, and housekeeping
  // Runs at ~100Hz on Core 0
  // -------------------------------------------------------------------------
  static int rx_fail = 0;   // Counter for RX failsafe
  static int bat_div = 0;   // Divider for battery check (1Hz)
  static int led_blink = 0; // Counter for LED blinking

  while (1) {
    // Check if receiver is connected (timeout-based)
    bool rx_instant = rx_is_connected();
    rx_fail = rx_instant ? 0 : rx_fail + 1;
    bool rx_ok = (rx_fail < 20); // ~200ms timeout

    if (!rx_ok && system_armed) {
      printf("!!! RX FAILSAFE - DISARMING !!!\n");
    }

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
        mixer_arm(true);
        // blackbox_clear();      // Start fresh log (disabled)
        // blackbox_start();
        printf(">>> ARMED - MOTORS LIVE! <<<\n");
      }
    } else {
      if (system_armed) {
        system_armed = false;
        mixer_arm(false);
        // blackbox_stop();  // (disabled)
        printf(">>> DISARMED <<<\n");
      }
      // Clear error state when arm switch is turned off
      if (error_state && !arm_sw) {
        error_state = false;
        printf("Error state cleared\n");
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
        printf("!!! LOW BATTERY: %d mV - LAND NOW! !!!\n", debug_vbat);
      }
    }

    // -----------------------------------------------------------------------
    // LED Status Indication
    // - Blinking = low battery warning
    // - Solid on = armed
    // - Off = disarmed
    // -----------------------------------------------------------------------
    bool low_bat = (debug_vbat > 0 && debug_vbat < sys_cfg.low_bat_threshold);
    if (low_bat) {
      // Fast blink for low battery
      if (++led_blink >= 25) {
        led_blink = 0;
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
      }
    } else {
      led_blink = 0;
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
      printf("!!! EMERGENCY STOP ACTIVATED !!!\n");
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // ~100Hz loop rate
  }
}
