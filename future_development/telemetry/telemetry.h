#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

// Telemetry packet structure - must match ground station
typedef struct __attribute__((packed)) {
  float roll;
  float pitch;
  float yaw;
  float battery_v;
  float latitude;
  float longitude;
  float altitude;
  uint8_t armed;
  uint8_t gps_fix;
} telemetry_packet_t;

// Initialize nRF24L01 telemetry
void telemetry_init(void);

// Send telemetry packet (non-blocking)
void telemetry_send(const telemetry_packet_t *packet);

// Check if telemetry is ready to send
bool telemetry_ready(void);

#endif
