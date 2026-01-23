/**
 * VL53L0X Minimal Driver for ESP32 (ESP-IDF)
 * Based on ST API (STSW-IMG005)
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Configuration - Change these as needed
#define VL53L0X_ADDR 0x29
#define VL53L0X_I2C_PORT I2C_NUM_0
#define VL53L0X_SDA 21
#define VL53L0X_SCL 22

typedef struct {
  uint8_t addr;
  uint8_t stop_var;
  bool ready;
} vl53l0x_t;

esp_err_t vl53l0x_i2c_init(void);
esp_err_t vl53l0x_init(vl53l0x_t *dev);
esp_err_t vl53l0x_read_single(vl53l0x_t *dev, uint16_t *mm);
esp_err_t vl53l0x_start_continuous(vl53l0x_t *dev);
esp_err_t vl53l0x_read_continuous(vl53l0x_t *dev, uint16_t *mm);
esp_err_t vl53l0x_stop_continuous(vl53l0x_t *dev);

#endif
