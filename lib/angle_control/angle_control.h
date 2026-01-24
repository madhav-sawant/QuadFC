/**
 * @file angle_control.h
 * @brief Angle PI controller for roll and pitch axes
 */

#ifndef ANGLE_CONTROL_H
#define ANGLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float target_roll_rate;
  float target_pitch_rate;
} angle_output_t;

void angle_control_init(void);

void angle_control_update(float target_roll, float target_pitch,
                          float actual_roll, float actual_pitch, float dt_sec,
                          bool armed, uint16_t throttle);

const angle_output_t *angle_control_get_output(void);

#endif // ANGLE_CONTROL_H

