#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdbool.h>

// Initialize Wi-Fi in AP mode and start the HTTP server
void webserver_init(void);

// Set target rates for live display (called from control loop)
void webserver_set_rate_targets(float roll_rate, float pitch_rate);

// Set target angles for live display (called from control loop)
void webserver_set_angle_targets(float roll_angle, float pitch_angle);

// Set error message to display on webserver (called when errors occur)
void webserver_set_error(const char *msg);

#endif // WEBSERVER_H
