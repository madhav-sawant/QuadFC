#ifndef ADC_H
#define ADC_H

#include "esp_adc/adc_oneshot.h"
#include <stdint.h>

// ADC1 Channel 4 (GPIO 32)
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_4
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_WIDTH ADC_BITWIDTH_12

// Calibration: R1=98.1k, R2=32.2k
// Adjusted to 4.11 for ADC non-linearity (reads ~0.15V low for safety)
#define VOLTAGE_SCALE_MV 41100 // 4.11 * 10000
#define VOLTAGE_SCALE_DIV 10000
#define VOLTAGE_OFFSET_MV 0

void adc_init(void);
uint16_t adc_read_battery_voltg(void);

#endif // ADC_H
