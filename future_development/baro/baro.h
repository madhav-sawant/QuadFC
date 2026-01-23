#ifndef BARO_H
#define BARO_H

#include <esp_err.h>
#include <stdbool.h>

typedef struct {
  float pressure_pa;
  float temperature_c;
  float altitude_m;
} baro_data_t;

// Initialize the BMP280 sensor
// Assumes I2C driver is already installed on the bus
esp_err_t baro_init(void);

// Read latest data from sensor
void baro_read(void);

// Get the latest data
const baro_data_t *baro_get_data(void);

#endif // BARO_H
