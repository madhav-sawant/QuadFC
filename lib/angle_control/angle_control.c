/**
 * @file angle_control.c
 * @brief Angle PI controller implementation
 */

#include "angle_control.h"
#include "../config/config.h"
#include <math.h>

// Internal state
static float i_roll_accum = 0.0f;
static float i_pitch_accum = 0.0f;
static angle_output_t output;

// Constants from main.c
#define MAX_ANGLE_I 30.0f
#define MAX_ANGLE_RATE 150.0f
#define ANGLE_I_THROTTLE_MIN 1200

void angle_control_init(void) {
  i_roll_accum = 0.0f;
  i_pitch_accum = 0.0f;
  output.target_roll_rate = 0.0f;
  output.target_pitch_rate = 0.0f;
}

void angle_control_update(float target_roll, float target_pitch,
                          float actual_roll, float actual_pitch, float dt_sec,
                          bool armed, uint16_t throttle) {
  float roll_error = target_roll - actual_roll;
  float pitch_error = target_pitch - actual_pitch;

  // I-term logic
  if (!armed) {
    i_roll_accum = 0.0f;
    i_pitch_accum = 0.0f;
  } else if (sys_cfg.angle_ki > 0.0f && throttle > ANGLE_I_THROTTLE_MIN) {
    // Only accumulate if armed and throttle is high enough
    float max_i_accum = MAX_ANGLE_I / sys_cfg.angle_ki;

    i_roll_accum += roll_error * dt_sec;
    if (i_roll_accum > max_i_accum)
      i_roll_accum = max_i_accum;
    if (i_roll_accum < -max_i_accum)
      i_roll_accum = -max_i_accum;

    i_pitch_accum += pitch_error * dt_sec;
    if (i_pitch_accum > max_i_accum)
      i_pitch_accum = max_i_accum;
    if (i_pitch_accum < -max_i_accum)
      i_pitch_accum = -max_i_accum;
  } else {
    // Hold I-term (don't clear it mid-flight if throttle drops momentarily, but don't accumulate)
    // Actually main.c logic was: if armed but low throttle -> hold? 
    // Re-reading main.c: "else { angle_i = 0 }" was applied if (!armed).
    // The "else if" managed the active accumulation.
    // Wait, main.c had a subtle logic: 
    // if (!armed) { reset } 
    // else if (ki > 0 && thr > 1200) { accumulate }
    // else { reset } <--- This 3rd branch in main.c line 157 resets it if high throttle condition fails!
    // Let's replicate main.c exactly to avoid behavior changes.
    i_roll_accum = 0.0f;
    i_pitch_accum = 0.0f;
  }

  // Calculate Target Rate = P + I
  output.target_roll_rate =
      sys_cfg.angle_kp * roll_error + sys_cfg.angle_ki * i_roll_accum;
  output.target_pitch_rate =
      sys_cfg.angle_kp * pitch_error + sys_cfg.angle_ki * i_pitch_accum;

  // Clamp Output Rates
  if (output.target_roll_rate > MAX_ANGLE_RATE)
    output.target_roll_rate = MAX_ANGLE_RATE;
  if (output.target_roll_rate < -MAX_ANGLE_RATE)
    output.target_roll_rate = -MAX_ANGLE_RATE;

  if (output.target_pitch_rate > MAX_ANGLE_RATE)
    output.target_pitch_rate = MAX_ANGLE_RATE;
  if (output.target_pitch_rate < -MAX_ANGLE_RATE)
    output.target_pitch_rate = -MAX_ANGLE_RATE;
}

const angle_output_t *angle_control_get_output(void) { return &output; }
