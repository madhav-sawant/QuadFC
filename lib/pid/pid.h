#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float output_limit;
  float integral_limit;
  float integral;
  float prev_measurement;    // For derivative-on-measurement
  float filtered_derivative; // Low-pass filtered D-term
  bool integral_frozen;      // Freeze I-term accumulation (e.g., low throttle)
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit);

float pid_calculate(pid_controller_t *pid, float setpoint, float measurement,
                    float dt_sec);

// Freeze/unfreeze integral accumulation (prevents windup during ground idle)
void pid_freeze_integral(pid_controller_t *pid, bool freeze);

// Reset PID state (integral, derivative, previous measurement)
void pid_reset(pid_controller_t *pid);

#endif // PID_H
