/**
 * @file imu.c
 * @brief MPU6050 IMU driver with complementary filter for angle estimation
 */

#include "imu.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdio.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * MPU6050 Register Definitions
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define MPU6050_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_USER_CTRL 0x6A
#define REG_INT_PIN_CFG 0x37
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43
#define REG_WHO_AM_I 0x75

/* ─────────────────────────────────────────────────────────────────────────────
 * Configuration Constants
 * ─────────────────────────────────────────────────────────────────────────────
 */
// I2C Configuration
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000

// MPU6050 Settings
#define DLPF_CFG_98HZ 0x02          // 98Hz bandwidth, ~4ms delay
#define FS_SEL_2000 0x18            // ±2000 deg/s gyro range
#define AFS_SEL_2G 0x00             // ±2g accelerometer range
#define GYRO_SCALE_FACTOR 16.4f     // LSB/(deg/s) for ±2000 range
#define ACCEL_SCALE_FACTOR 16384.0f // LSB/g for ±2g range

// Sensor Fusion
#define COMPLEMENTARY_ALPHA 0.996f  // 99.6% gyro, 0.4% accel (was 0.9996 - too gyro-heavy, caused drift)
#define GYRO_LPF_ALPHA 0.70f        // Gyro software LPF (for PID)
#define ACCEL_LPF_ALPHA 0.15f       // Accel software LPF
#define MAX_ANGLE_RATE_DPS 500.0f   // Spike filter limit
#define RAD_TO_DEG 57.2957795f

/* ─────────────────────────────────────────────────────────────────────────────
 * Static Variables
 * ─────────────────────────────────────────────────────────────────────────────
 */
static imu_data_t imu_state;

// Calibration values
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static float accel_offset_pitch = 0.0f;
static float accel_offset_roll = 0.0f;
static float accel_scale_factor = 1.0f;

// Software LPF state
static float gyro_filtered_x = 0.0f;
static float gyro_filtered_y = 0.0f;
static float gyro_filtered_z = 0.0f;
static float accel_filtered_x = 0.0f;
static float accel_filtered_y = 0.0f;
static float accel_filtered_z = 0.0f;
static bool filter_initialized = false;

/* ─────────────────────────────────────────────────────────────────────────────
 * I2C Helper Functions
 * ─────────────────────────────────────────────────────────────────────────────
 */
static uint32_t i2c_error_count = 0;

static esp_err_t write_register(uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) i2c_error_count++;
  return ret;
}

static esp_err_t read_registers(uint8_t start_reg, uint8_t *buffer,
                                size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, start_reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, buffer, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, buffer + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) i2c_error_count++;
  return ret;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Initialization
 * ─────────────────────────────────────────────────────────────────────────────
 */
esp_err_t imu_init(void) {
  uint8_t who_am_i;
  esp_err_t ret = read_registers(REG_WHO_AM_I, &who_am_i, 1);

  // Initialize I2C if not already done
  if (ret != ESP_OK) {
    printf("IMU: Initializing I2C...\n");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  }

  // Verify MPU6050 presence
  if (read_registers(REG_WHO_AM_I, &who_am_i, 1) != ESP_OK ||
      who_am_i != MPU6050_ADDR) {
    printf("IMU: MPU6050 not found!\n");
    return ESP_FAIL;
  }
  printf("IMU: MPU6050 detected\n");

  // Configure MPU6050
  write_register(REG_PWR_MGMT_1, 0x01);  // PLL with X gyro reference
  write_register(REG_USER_CTRL, 0x00);   // Disable I2C master mode
  write_register(REG_INT_PIN_CFG, 0x02); // Enable I2C bypass
  write_register(REG_CONFIG, DLPF_CFG_98HZ);
  write_register(REG_GYRO_CONFIG, FS_SEL_2000);
  write_register(REG_ACCEL_CONFIG, AFS_SEL_2G);

  return ESP_OK;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Calibration Functions
 * ─────────────────────────────────────────────────────────────────────────────
 */
void imu_calibrate_gyro(void) {
  const int samples = 1000;
  float sum_x = 0, sum_y = 0, sum_z = 0;
  uint8_t buffer[6];

  for (int i = 0; i < samples; i++) {
    if (read_registers(REG_GYRO_XOUT_H, buffer, 6) == ESP_OK) {
      sum_x += (int16_t)((buffer[0] << 8) | buffer[1]);
      sum_y += (int16_t)((buffer[2] << 8) | buffer[3]);
      sum_z += (int16_t)((buffer[4] << 8) | buffer[5]);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  gyro_bias_x = (sum_x / samples) / GYRO_SCALE_FACTOR;
  gyro_bias_y = (sum_y / samples) / GYRO_SCALE_FACTOR;
  gyro_bias_z = (sum_z / samples) / GYRO_SCALE_FACTOR;
}

void imu_calibrate_accel(void) {
  const int samples = 1000;
  float sum_pitch = 0, sum_roll = 0, sum_z = 0;
  uint8_t buffer[6];

  printf("IMU: Calibrating accelerometer...\n");

  for (int i = 0; i < samples; i++) {
    if (read_registers(REG_ACCEL_XOUT_H, buffer, 6) == ESP_OK) {
      float ax = (int16_t)((buffer[0] << 8) | buffer[1]) / ACCEL_SCALE_FACTOR;
      float ay = (int16_t)((buffer[2] << 8) | buffer[3]) / ACCEL_SCALE_FACTOR;
      float az = (int16_t)((buffer[4] << 8) | buffer[5]) / ACCEL_SCALE_FACTOR;

      sum_pitch += atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
      sum_roll += atan2f(ay, az) * RAD_TO_DEG;
      sum_z += az;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  accel_offset_pitch = sum_pitch / samples;
  accel_offset_roll = sum_roll / samples;

  // Calculate scale factor (Z should read 1.0g when level)
  float avg_z = sum_z / samples;
  if (avg_z > 0.5f && avg_z < 1.5f) {
    accel_scale_factor = 1.0f / avg_z;
  }
  printf("IMU: Accel calibration complete (scale=%.3f)\n", accel_scale_factor);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Main Read Function
 * ─────────────────────────────────────────────────────────────────────────────
 */
void imu_read(float dt_sec) {
  static bool first_read = true;
  uint8_t buffer[14];

  if (read_registers(REG_ACCEL_XOUT_H, buffer, 14) != ESP_OK) {
    return;
  }

  // Parse raw sensor data
  int16_t ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
  int16_t ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
  int16_t az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
  int16_t gx_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
  int16_t gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
  int16_t gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

  // Convert to physical units and apply calibration
  float accel_raw_x = (ax_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float accel_raw_y = (ay_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float accel_raw_z = (az_raw / ACCEL_SCALE_FACTOR) * accel_scale_factor;
  float gyro_raw_x = (gx_raw / GYRO_SCALE_FACTOR) - gyro_bias_x;
  float gyro_raw_y = (gy_raw / GYRO_SCALE_FACTOR) - gyro_bias_y;
  float gyro_raw_z = (gz_raw / GYRO_SCALE_FACTOR) - gyro_bias_z;

  // Apply software low-pass filters
  if (!filter_initialized) {
    accel_filtered_x = accel_raw_x;
    accel_filtered_y = accel_raw_y;
    accel_filtered_z = accel_raw_z;
    gyro_filtered_x = gyro_raw_x;
    gyro_filtered_y = gyro_raw_y;
    gyro_filtered_z = gyro_raw_z;
    filter_initialized = true;
  } else {
    accel_filtered_x = ACCEL_LPF_ALPHA * accel_raw_x +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_x;
    accel_filtered_y = ACCEL_LPF_ALPHA * accel_raw_y +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_y;
    accel_filtered_z = ACCEL_LPF_ALPHA * accel_raw_z +
                       (1.0f - ACCEL_LPF_ALPHA) * accel_filtered_z;
    gyro_filtered_x =
        GYRO_LPF_ALPHA * gyro_raw_x + (1.0f - GYRO_LPF_ALPHA) * gyro_filtered_x;
    gyro_filtered_y =
        GYRO_LPF_ALPHA * gyro_raw_y + (1.0f - GYRO_LPF_ALPHA) * gyro_filtered_y;
    gyro_filtered_z =
        GYRO_LPF_ALPHA * gyro_raw_z + (1.0f - GYRO_LPF_ALPHA) * gyro_filtered_z;
  }

  // Store filtered values for PID
  imu_state.accel_x_g = accel_filtered_x;
  imu_state.accel_y_g = accel_filtered_y;
  imu_state.accel_z_g = accel_filtered_z;
  imu_state.gyro_x_dps = gyro_filtered_x;
  imu_state.gyro_y_dps = gyro_filtered_y;
  imu_state.gyro_z_dps = gyro_filtered_z;

  // Calculate accelerometer angles
  float accel_pitch =
      atan2f(accel_filtered_x, sqrtf(accel_filtered_y * accel_filtered_y +
                                     accel_filtered_z * accel_filtered_z)) *
      RAD_TO_DEG;
  float accel_roll = atan2f(accel_filtered_y, accel_filtered_z) * RAD_TO_DEG;

  // Apply calibration offsets
  accel_pitch -= accel_offset_pitch;
  accel_roll -= accel_offset_roll;

  // Complementary filter for angle estimation
  if (first_read) {
    imu_state.pitch_deg = accel_pitch;
    imu_state.roll_deg = accel_roll;
    first_read = false;
    printf("IMU: Initialized (Roll=%.1f, Pitch=%.1f)\n", imu_state.roll_deg,
           imu_state.pitch_deg);
  } else {
    float prev_pitch = imu_state.pitch_deg;
    float prev_roll = imu_state.roll_deg;

    // Use RAW gyro for angle integration (faster response)
    float new_pitch =
        COMPLEMENTARY_ALPHA * (prev_pitch + gyro_raw_y * dt_sec) +
        (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
    float new_roll =
        COMPLEMENTARY_ALPHA * (prev_roll + gyro_raw_x * dt_sec) +
        (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;

    // Spike filter: limit angle rate of change
    float max_delta = MAX_ANGLE_RATE_DPS * dt_sec;
    float pitch_delta = new_pitch - prev_pitch;
    float roll_delta = new_roll - prev_roll;

    if (pitch_delta > max_delta)
      pitch_delta = max_delta;
    if (pitch_delta < -max_delta)
      pitch_delta = -max_delta;
    if (roll_delta > max_delta)
      roll_delta = max_delta;
    if (roll_delta < -max_delta)
      roll_delta = -max_delta;

    imu_state.pitch_deg = prev_pitch + pitch_delta;
    imu_state.roll_deg = prev_roll + roll_delta;
  }
}

const imu_data_t *imu_get_data(void) { return &imu_state; }

/* ─────────────────────────────────────────────────────────────────────────────
 * NVS Storage
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define NVS_NAMESPACE "imu_cal"
#define NVS_MAGIC 0xCAFE1234

bool imu_calibration_load_from_nvs(void) {
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
    printf("IMU: No saved calibration\n");
    return false;
  }

  uint32_t magic = 0;
  size_t len = sizeof(uint32_t);
  if (nvs_get_blob(handle, "magic", &magic, &len) != ESP_OK ||
      magic != NVS_MAGIC) {
    nvs_close(handle);
    return false;
  }

  len = sizeof(float);
  nvs_get_blob(handle, "gyro_bias_x", &gyro_bias_x, &len);
  nvs_get_blob(handle, "gyro_bias_y", &gyro_bias_y, &len);
  nvs_get_blob(handle, "gyro_bias_z", &gyro_bias_z, &len);
  nvs_get_blob(handle, "accel_off_p", &accel_offset_pitch, &len);
  nvs_get_blob(handle, "accel_off_r", &accel_offset_roll, &len);
  nvs_get_blob(handle, "accel_scale", &accel_scale_factor, &len);

  nvs_close(handle);
  printf("IMU: Calibration loaded from NVS\n");
  return true;
}

void imu_calibration_save_to_nvs(void) {
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
    return;
  }

  nvs_set_blob(handle, "gyro_bias_x", &gyro_bias_x, sizeof(float));
  nvs_set_blob(handle, "gyro_bias_y", &gyro_bias_y, sizeof(float));
  nvs_set_blob(handle, "gyro_bias_z", &gyro_bias_z, sizeof(float));
  nvs_set_blob(handle, "accel_off_p", &accel_offset_pitch, sizeof(float));
  nvs_set_blob(handle, "accel_off_r", &accel_offset_roll, sizeof(float));
  nvs_set_blob(handle, "accel_scale", &accel_scale_factor, sizeof(float));

  uint32_t magic = NVS_MAGIC;
  nvs_set_blob(handle, "magic", &magic, sizeof(uint32_t));

  nvs_commit(handle);
  nvs_close(handle);
  printf("IMU: Calibration saved to NVS\n");
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Utility Functions
 * ─────────────────────────────────────────────────────────────────────────────
 */
void imu_get_calibration(imu_calibration_t *cal) {
  if (cal == NULL)
    return;
  cal->gyro_bias_x = gyro_bias_x;
  cal->gyro_bias_y = gyro_bias_y;
  cal->gyro_bias_z = gyro_bias_z;
  cal->accel_offset_pitch = accel_offset_pitch;
  cal->accel_offset_roll = accel_offset_roll;
}

void imu_print_calibration(void) {
  printf("\n=== IMU Calibration ===\n");
  printf("Gyro Bias: X=%.4f Y=%.4f Z=%.4f\n", gyro_bias_x, gyro_bias_y,
         gyro_bias_z);
  printf("Accel Offset: Pitch=%.4f Roll=%.4f\n", accel_offset_pitch,
         accel_offset_roll);
  printf("Accel Scale: %.4f\n", accel_scale_factor);
}

uint32_t imu_get_i2c_errors(void) {
  return i2c_error_count;
}

/**
 * Set current surface as LEVEL when arming
 * 
 * Whatever the accelerometer reads right now becomes the new "zero".
 * This allows flying from ANY surface without drift.
 */
void imu_set_level_on_arm(void) {
  uint8_t buffer[6];
  if (read_registers(REG_ACCEL_XOUT_H, buffer, 6) == ESP_OK) {
    float ax = (int16_t)((buffer[0] << 8) | buffer[1]) / ACCEL_SCALE_FACTOR;
    float ay = (int16_t)((buffer[2] << 8) | buffer[3]) / ACCEL_SCALE_FACTOR;
    float az = (int16_t)((buffer[4] << 8) | buffer[5]) / ACCEL_SCALE_FACTOR;
    
    // Calculate RAW accelerometer angle (no offset applied)
    float raw_accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float raw_accel_roll = atan2f(ay, az) * RAD_TO_DEG;
    
    // Update calibration offset = current raw reading
    // This makes current surface the new "zero"
    accel_offset_pitch = raw_accel_pitch;
    accel_offset_roll = raw_accel_roll;
    
    // Set angle to ZERO (current surface is now "level")
    imu_state.pitch_deg = 0.0f;
    imu_state.roll_deg = 0.0f;
    
    // printf removed for timing - runs in arm path
  }
}
