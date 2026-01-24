#ifndef RATE_CONTROL_H
#define RATE_CONTROL_H

#include "../config/config.h"
#include <stdbool.h>

#define RATE_LOOP_DT_SEC 0.004f

typedef struct {
  float roll;
  float pitch;
  float yaw;
} rate_output_t;

void rate_control_init(void);
void rate_control_update(float desired_roll_rate, float desired_pitch_rate,
                         float desired_yaw_rate, float gyro_roll_rate,
                         float gyro_pitch_rate, float gyro_yaw_rate);
const rate_output_t *rate_control_get_output(void);
void rate_control_freeze_integral(bool freeze);

#endif // RATE_CONTROL_H

