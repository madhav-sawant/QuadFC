#ifndef IMU_H
#define IMU_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float roll_deg;
  float pitch_deg;
} imu_data_t;

esp_err_t imu_init(void);
void imu_read(float dt_sec);
const imu_data_t *imu_get_data(void);
void imu_calibrate_gyro(void);
void imu_calibrate_accel(void);

// NVS Calibration Storage (uses separate namespace from PID config)
bool imu_calibration_load_from_nvs(void);
void imu_calibration_save_to_nvs(void);

// Calibration data structure for external access
typedef struct {
  float gyro_bias_x;
  float gyro_bias_y;
  float gyro_bias_z;
  float accel_offset_pitch;
  float accel_offset_roll;
} imu_calibration_t;

// Get current calibration values (non-blocking, safe to call anytime)
void imu_get_calibration(imu_calibration_t *cal);

// Print calibration values to console
void imu_print_calibration(void);

// Get diagnostic counters
uint32_t imu_get_i2c_errors(void);

// Auto-level on arm: captures current angle as "level"
// Call this when arming so drone can take off from any surface
void imu_set_level_on_arm(void);

#endif // IMU_H
