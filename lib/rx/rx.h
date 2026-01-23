#ifndef RX_H
#define RX_H

#include <stdbool.h>
#include <stdint.h>

#define RX_PIN 33
#define RX_CHANNEL_COUNT 8

// Standard PPM ranges
#define RX_MIN_US 900
#define RX_MAX_US 2100
#define RX_SYNC_MIN_US 2100 // Sync pause is usually > 2ms

void rx_init(void);
bool rx_is_connected(void); // Returns true if valid frames are being received
uint16_t rx_get_channel(uint8_t channel_index); // 0-indexed
void rx_get_all(uint16_t *channels);

#endif // RX_H
