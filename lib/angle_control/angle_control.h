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

/**
 * @brief Initialize the angle controller
 */
void angle_control_init(void);

/**
 * @brief Update the angle PI controller
 *
 * @param target_roll Target roll angle in degrees
 * @param target_pitch Target pitch angle in degrees
 * @param actual_roll Actual roll angle from IMU
 * @param actual_pitch Actual pitch angle from IMU
 * @param dt_sec Time step in seconds
 * @param armed System arming state (for I-term management)
 * @param throttle Current throttle value (for I-term management)
 */
void angle_control_update(float target_roll, float target_pitch,
                          float actual_roll, float actual_pitch, float dt_sec,
                          bool armed, uint16_t throttle);

/**
 * @brief Get the output target rates from the angle controller
 * @return Pointer to output structure
 */
const angle_output_t *angle_control_get_output(void);

/**
 * @brief Get the current I-term accumulator values (for debugging)
 * @param roll_i Pointer to store roll I-term
 * @param pitch_i Pointer to store pitch I-term
 */
void angle_control_get_i_terms(float *roll_i, float *pitch_i);

#endif // ANGLE_CONTROL_H
