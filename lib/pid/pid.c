/**
 * @file pid.c
 * @brief Generic PID controller with D-term filtering and integral freeze
 */

#include "pid.h"
#include <stdbool.h>

#define D_TERM_LPF_ALPHA 0.85f // D-term low-pass filter coefficient (Higher = less filtering/lag)

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->output_limit = output_limit;
  pid->integral_limit = integral_limit;
  pid->integral = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->filtered_derivative = 0.0f;
  pid->integral_frozen = false;
}

float pid_calculate(pid_controller_t *pid, float setpoint, float measurement,
                    float dt_sec) {
  float error = setpoint - measurement;

  // P-term
  float p_out = pid->kp * error;

  // I-term (with freeze capability)
  if (!pid->integral_frozen) {
    pid->integral += error * dt_sec;
    if (pid->integral > pid->integral_limit)
      pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
      pid->integral = -pid->integral_limit;
  }
  float i_out = pid->ki * pid->integral;

  // D-term on measurement (avoids derivative kick on setpoint change)
  float raw_d = -(measurement - pid->prev_measurement) / dt_sec;
  pid->prev_measurement = measurement;

  // Low-pass filter the derivative
  pid->filtered_derivative =
      D_TERM_LPF_ALPHA * raw_d +
      (1.0f - D_TERM_LPF_ALPHA) * pid->filtered_derivative;
  float d_out = pid->kd * pid->filtered_derivative;

  // Sum and clamp
  float output = p_out + i_out + d_out;
  if (output > pid->output_limit)
    output = pid->output_limit;
  else if (output < -pid->output_limit)
    output = -pid->output_limit;

  return output;
}

void pid_freeze_integral(pid_controller_t *pid, bool freeze) {
  pid->integral_frozen = freeze;
}

void pid_reset(pid_controller_t *pid) {
  pid->integral = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->filtered_derivative = 0.0f;
}
