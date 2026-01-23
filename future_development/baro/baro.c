#include "baro.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define TAG "BARO"

// BMP280 I2C Address (Try 0x76 first, then 0x77)
#define BMP280_ADDR 0x76

// Registers
#define BMP280_REG_TEMP_XLSB 0xFC
#define BMP280_REG_TEMP_LSB 0xFB
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_PRESS_LSB 0xF8
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_ID 0xD0
#define BMP280_REG_CALIB 0x88

// Calibration Data
typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
} bmp280_calib_t;

static bmp280_calib_t calib;
static baro_data_t baro_state;
static bool initialized = false;

// I2C Helpers (Should be shared but defining here for isolation)
#define I2C_MASTER_NUM 0
#define I2C_MASTER_TIMEOUT_MS 1000

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t write_reg(uint8_t reg, uint8_t value) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

static void read_calibration_data(void) {
  uint8_t data[24];
  read_regs(BMP280_REG_CALIB, data, 24);

  calib.dig_T1 = (data[1] << 8) | data[0];
  calib.dig_T2 = (int16_t)((data[3] << 8) | data[2]);
  calib.dig_T3 = (int16_t)((data[5] << 8) | data[4]);
  calib.dig_P1 = (data[7] << 8) | data[6];
  calib.dig_P2 = (int16_t)((data[9] << 8) | data[8]);
  calib.dig_P3 = (int16_t)((data[11] << 8) | data[10]);
  calib.dig_P4 = (int16_t)((data[13] << 8) | data[12]);
  calib.dig_P5 = (int16_t)((data[15] << 8) | data[14]);
  calib.dig_P6 = (int16_t)((data[17] << 8) | data[16]);
  calib.dig_P7 = (int16_t)((data[19] << 8) | data[18]);
  calib.dig_P8 = (int16_t)((data[21] << 8) | data[20]);
  calib.dig_P9 = (int16_t)((data[23] << 8) | data[22]);
}

// Compensation functions from Bosch datasheet
static int32_t t_fine;
static float compensate_temp(int32_t adc_T) {
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) *
          ((int32_t)calib.dig_T2)) >>
         11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >>
           12) *
          ((int32_t)calib.dig_T3)) >>
         14;
  t_fine = var1 + var2;
  return (t_fine * 5 + 128) >> 8;
}

static float compensate_pressure(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
         ((var1 * (int64_t)calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
  if (var1 == 0)
    return 0;
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
  return (float)p / 256.0f;
}

esp_err_t baro_init(void) {
  uint8_t chip_id;
  esp_err_t ret = read_regs(BMP280_REG_ID, &chip_id, 1);
  if (ret != ESP_OK) {
    printf("BARO: Failed to communicate (Check I2C pins/power)\n");
    return ret;
  }

  if (chip_id != 0x58 && chip_id != 0x60) { // 0x58 is BMP280, 0x60 is BME280
    printf("BARO: Unrecognized Chip ID 0x%02x (Expected 0x58 or 0x60)\n",
           chip_id);
    return ESP_FAIL;
  }
  printf("BARO: Found BMP280/BME280 (ID 0x%02x)\n", chip_id);

  read_calibration_data();

  // Config: Standby 0.5ms, Filter 16, Spi disable
  write_reg(BMP280_REG_CONFIG, (0x00 << 5) | (0x04 << 2) | 0x00);

  // Ctrl Meas: Osrs_T x2, Osrs_P x16, Normal Mode
  // Osrs_T = 010 (x2) -> 0x40
  // Osrs_P = 101 (x16) -> 0x14
  // Mode = 11 (Normal) -> 0x03
  write_reg(BMP280_REG_CTRL_MEAS, 0x57);

  initialized = true;
  return ESP_OK;
}

void baro_read(void) {
  if (!initialized)
    return;

  uint8_t data[6];
  if (read_regs(BMP280_REG_PRESS_MSB, data, 6) == ESP_OK) {
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    float temp = compensate_temp(adc_T) / 100.0f;
    float pressure = compensate_pressure(adc_P);

    baro_state.temperature_c = temp;
    baro_state.pressure_pa = pressure;
    // Simple altitude calc: 44330 * (1 - (p/p0)^(1/5.255))
    // P0 = 101325 Pa (Sea Level Standard)
    baro_state.altitude_m =
        44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));
  }
}

const baro_data_t *baro_get_data(void) { return &baro_state; }
