/**
 * VL53L0X Laser Sensor Driver for ESP32
 * Based on ST API (STSW-IMG005)
 */
#include "vl53l0x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "VL53L0X"

// ============== I2C Helper Functions ==============

static esp_err_t write_reg(vl53l0x_t *dev, uint8_t reg, uint8_t val) {
  uint8_t data[2] = {reg, val};
  return i2c_master_write_to_device(VL53L0X_I2C_PORT, dev->addr, data, 2,
                                    pdMS_TO_TICKS(50));
}

static esp_err_t read_reg(vl53l0x_t *dev, uint8_t reg, uint8_t *val) {
  return i2c_master_write_read_device(VL53L0X_I2C_PORT, dev->addr, &reg, 1, val,
                                      1, pdMS_TO_TICKS(50));
}

static esp_err_t read_reg16(vl53l0x_t *dev, uint8_t reg, uint16_t *val) {
  uint8_t data[2];
  esp_err_t ret = i2c_master_write_read_device(
      VL53L0X_I2C_PORT, dev->addr, &reg, 1, data, 2, pdMS_TO_TICKS(50));
  *val = (data[0] << 8) | data[1];
  return ret;
}

static esp_err_t write_multi(vl53l0x_t *dev, uint8_t reg, uint8_t *data,
                             size_t len) {
  uint8_t buf[len + 1];
  buf[0] = reg;
  memcpy(buf + 1, data, len);
  return i2c_master_write_to_device(VL53L0X_I2C_PORT, dev->addr, buf, len + 1,
                                    pdMS_TO_TICKS(50));
}

static esp_err_t read_multi(vl53l0x_t *dev, uint8_t reg, uint8_t *data,
                            size_t len) {
  return i2c_master_write_read_device(VL53L0X_I2C_PORT, dev->addr, &reg, 1,
                                      data, len, pdMS_TO_TICKS(50));
}

// ============== Public Functions ==============

esp_err_t vl53l0x_i2c_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = VL53L0X_SDA,
      .scl_io_num = VL53L0X_SCL,
      .sda_pullup_en =
          GPIO_PULLUP_DISABLE, // External pull-ups likely present on modules
      .scl_pullup_en = GPIO_PULLUP_DISABLE,
      .master.clk_speed = 400000}; // Restored to 400kHz
  i2c_param_config(VL53L0X_I2C_PORT, &conf);
  return i2c_driver_install(VL53L0X_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t vl53l0x_init(vl53l0x_t *dev) {
  uint8_t tmp, spad_count, spad_map[6];

  dev->addr = VL53L0X_ADDR;
  dev->ready = false;
  vTaskDelay(pdMS_TO_TICKS(50));

  // Check sensor ID
  read_reg(dev, 0xC0, &tmp);
  if (tmp != 0xEE) {
    ESP_LOGE(TAG, "Sensor not found (ID: 0x%02X)", tmp);
    return ESP_ERR_NOT_FOUND;
  }
  ESP_LOGI(TAG, "VL53L0X detected");

  // Enable 2.8V mode
  read_reg(dev, 0x89, &tmp);
  write_reg(dev, 0x89, tmp | 0x01);

  // Standard initialization sequence
  write_reg(dev, 0x88, 0x00);
  write_reg(dev, 0x80, 0x01);
  write_reg(dev, 0xFF, 0x01);
  write_reg(dev, 0x00, 0x00);
  read_reg(dev, 0x91, &dev->stop_var);
  write_reg(dev, 0x00, 0x01);
  write_reg(dev, 0xFF, 0x00);
  write_reg(dev, 0x80, 0x00);

  // Disable MSRC and pre-range limit checks
  read_reg(dev, 0x60, &tmp);
  write_reg(dev, 0x60, tmp | 0x12);

  // Set signal rate limit to 0.25 MCPS
  write_reg(dev, 0x44, 0x00);
  write_reg(dev, 0x45, 0x20);
  write_reg(dev, 0x01, 0xFF);

  // Get SPAD info
  write_reg(dev, 0x80, 0x01);
  write_reg(dev, 0xFF, 0x01);
  write_reg(dev, 0x00, 0x00);
  write_reg(dev, 0xFF, 0x06);
  read_reg(dev, 0x83, &tmp);
  write_reg(dev, 0x83, tmp | 0x04);
  write_reg(dev, 0xFF, 0x07);
  write_reg(dev, 0x81, 0x01);
  write_reg(dev, 0x80, 0x01);
  write_reg(dev, 0x94, 0x6B);
  write_reg(dev, 0x83, 0x00);

  // Wait for SPAD info ready
  for (int i = 0; i < 100 && tmp == 0; i++) {
    read_reg(dev, 0x83, &tmp);
    vTaskDelay(1);
  }

  write_reg(dev, 0x83, 0x01);
  read_reg(dev, 0x92, &tmp);
  spad_count = tmp & 0x7F;

  // Restore registers
  write_reg(dev, 0x81, 0x00);
  write_reg(dev, 0xFF, 0x06);
  read_reg(dev, 0x83, &tmp);
  write_reg(dev, 0x83, tmp & ~0x04);
  write_reg(dev, 0xFF, 0x01);
  write_reg(dev, 0x00, 0x01);
  write_reg(dev, 0xFF, 0x00);
  write_reg(dev, 0x80, 0x00);

  // Configure SPADs
  read_multi(dev, 0xB0, spad_map, 6);
  write_reg(dev, 0xFF, 0x01);
  write_reg(dev, 0x4F, 0x00);
  write_reg(dev, 0x4E, 0x2C);
  write_reg(dev, 0xFF, 0x00);
  write_reg(dev, 0xB6, 0xB4);

  uint8_t first_spad = (tmp >> 7) ? 12 : 0;
  uint8_t enabled = 0;
  for (int i = 0; i < 48; i++) {
    if (i < first_spad || enabled == spad_count)
      spad_map[i / 8] &= ~(1 << (i % 8));
    else if ((spad_map[i / 8] >> (i % 8)) & 1)
      enabled++;
  }
  write_multi(dev, 0xB0, spad_map, 6);

  // Load tuning settings (from ST API)
  static const uint8_t tuning[] = {
      0xFF, 0x01, 0x00, 0x00, 0xFF, 0x00, 0x09, 0x00, 0x10, 0x00, 0x11, 0x00,
      0x24, 0x01, 0x25, 0xFF, 0x75, 0x00, 0xFF, 0x01, 0x4E, 0x2C, 0x48, 0x00,
      0x30, 0x20, 0xFF, 0x00, 0x30, 0x09, 0x54, 0x00, 0x31, 0x04, 0x32, 0x03,
      0x40, 0x83, 0x46, 0x25, 0x60, 0x00, 0x27, 0x00, 0x50, 0x06, 0x51, 0x00,
      0x52, 0x96, 0x56, 0x08, 0x57, 0x30, 0x61, 0x00, 0x62, 0x00, 0x64, 0x00,
      0x65, 0x00, 0x66, 0xA0, 0xFF, 0x01, 0x22, 0x32, 0x47, 0x14, 0x49, 0xFF,
      0x4A, 0x00, 0xFF, 0x00, 0x7A, 0x0A, 0x7B, 0x00, 0x78, 0x21, 0xFF, 0x01,
      0x23, 0x34, 0x42, 0x00, 0x44, 0xFF, 0x45, 0x26, 0x46, 0x05, 0x40, 0x40,
      0x0E, 0x06, 0x20, 0x1A, 0x43, 0x40, 0xFF, 0x00, 0x34, 0x03, 0x35, 0x44,
      0xFF, 0x01, 0x31, 0x04, 0x4B, 0x09, 0x4C, 0x05, 0x4D, 0x04, 0xFF, 0x00,
      0x44, 0x00, 0x45, 0x20, 0x47, 0x08, 0x48, 0x28, 0x67, 0x00, 0x70, 0x04,
      0x71, 0x01, 0x72, 0xFE, 0x76, 0x00, 0x77, 0x00, 0xFF, 0x01, 0x0D, 0x01,
      0xFF, 0x00, 0x80, 0x01, 0x01, 0xF8, 0xFF, 0x01, 0x8E, 0x01, 0x00, 0x01,
      0xFF, 0x00, 0x80, 0x00};
  for (size_t i = 0; i < sizeof(tuning); i += 2)
    write_reg(dev, tuning[i], tuning[i + 1]);

  // Configure GPIO interrupt
  write_reg(dev, 0x0A, 0x04);
  read_reg(dev, 0x84, &tmp);
  write_reg(dev, 0x84, tmp & ~0x10);
  write_reg(dev, 0x0B, 0x01);

  // VHV calibration
  write_reg(dev, 0x01, 0x01);
  write_reg(dev, 0x00, 0x41);
  for (int i = 0; i < 100 && !(tmp & 0x07); i++) {
    read_reg(dev, 0x13, &tmp);
    vTaskDelay(10);
  }
  write_reg(dev, 0x0B, 0x01);
  write_reg(dev, 0x00, 0x00);

  // Phase calibration
  write_reg(dev, 0x01, 0x02);
  write_reg(dev, 0x00, 0x41);
  for (int i = 0; i < 100 && !(tmp & 0x07); i++) {
    read_reg(dev, 0x13, &tmp);
    vTaskDelay(10);
  }
  write_reg(dev, 0x0B, 0x01);
  write_reg(dev, 0x00, 0x00);
  write_reg(dev, 0x01, 0xE8);

  dev->ready = true;
  ESP_LOGI(TAG, "Initialization complete");
  return ESP_OK;
}

esp_err_t vl53l0x_read_single(vl53l0x_t *dev, uint16_t *mm) {
  uint8_t status;

  // Start measurement
  write_reg(dev, 0x80, 0x01);
  write_reg(dev, 0xFF, 0x01);
  write_reg(dev, 0x00, 0x00);
  write_reg(dev, 0x91, dev->stop_var);
  write_reg(dev, 0x00, 0x01);
  write_reg(dev, 0xFF, 0x00);
  write_reg(dev, 0x80, 0x00);
  write_reg(dev, 0x00, 0x01);

  // Wait for start
  for (int i = 0; i < 100; i++) {
    read_reg(dev, 0x00, &status);
    if (!(status & 1))
      break;
    vTaskDelay(1);
  }

  // Wait for result
  for (int i = 0; i < 100; i++) {
    read_reg(dev, 0x13, &status);
    if (status & 0x07)
      break;
    vTaskDelay(10);
  }

  // Read distance and clear interrupt
  read_reg16(dev, 0x1E, mm);
  write_reg(dev, 0x0B, 0x01);

  if (*mm >= 8190)
    *mm = 0; // Out of range
  return ESP_OK;
}

esp_err_t vl53l0x_start_continuous(vl53l0x_t *dev) {
  write_reg(dev, 0x80, 0x01);
  write_reg(dev, 0xFF, 0x01);
  write_reg(dev, 0x00, 0x00);
  write_reg(dev, 0x91, dev->stop_var);
  write_reg(dev, 0x00, 0x01);
  write_reg(dev, 0xFF, 0x00);
  write_reg(dev, 0x80, 0x00);
  write_reg(dev, 0x00, 0x02); // Back-to-back mode
  return ESP_OK;
}

esp_err_t vl53l0x_read_continuous(vl53l0x_t *dev, uint16_t *mm) {
  uint8_t status;

  // Wait for data ready
  for (int i = 0; i < 100; i++) {
    read_reg(dev, 0x13, &status);
    if (status & 0x07)
      break;
    vTaskDelay(1);
  }

  // Read distance and clear interrupt
  read_reg16(dev, 0x1E, mm);
  write_reg(dev, 0x0B, 0x01);

  if (*mm >= 8190)
    *mm = 0; // Out of range
  return ESP_OK;
}

esp_err_t vl53l0x_stop_continuous(vl53l0x_t *dev) {
  write_reg(dev, 0x00, 0x01);
  write_reg(dev, 0xFF, 0x01);
  write_reg(dev, 0x00, 0x00);
  write_reg(dev, 0x91, 0x00);
  write_reg(dev, 0x00, 0x01);
  write_reg(dev, 0xFF, 0x00);
  return ESP_OK;
}
