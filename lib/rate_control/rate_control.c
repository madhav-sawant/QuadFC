/**
 * @file rate_control.c
 * @brief Rate PID controller for roll, pitch, and yaw axes
 */

#include "rate_control.h"
#include "../pid/pid.h"

static pid_controller_t pid_roll;
static pid_controller_t pid_pitch;
static pid_controller_t pid_yaw;
static rate_output_t output;

void rate_control_init(void) {
  pid_init(&pid_roll, sys_cfg.roll_kp, sys_cfg.roll_ki, sys_cfg.roll_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
  pid_init(&pid_pitch, sys_cfg.pitch_kp, sys_cfg.pitch_ki, sys_cfg.pitch_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
  pid_init(&pid_yaw, sys_cfg.yaw_kp, sys_cfg.yaw_ki, sys_cfg.yaw_kd,
           sys_cfg.rate_output_limit, sys_cfg.rate_integral_limit);
  output.roll = 0.0f;
  output.pitch = 0.0f;
  output.yaw = 0.0f;
}

void rate_control_update(float target_roll, float target_pitch,
                         float target_yaw, float gyro_roll, float gyro_pitch,
                         float gyro_yaw) {
  output.roll =
      pid_calculate(&pid_roll, target_roll, gyro_roll, RATE_LOOP_DT_SEC);
  output.pitch =
      pid_calculate(&pid_pitch, target_pitch, gyro_pitch, RATE_LOOP_DT_SEC);
  output.yaw = pid_calculate(&pid_yaw, target_yaw, gyro_yaw, RATE_LOOP_DT_SEC);
}

const rate_output_t *rate_control_get_output(void) { return &output; }

void rate_control_freeze_integral(bool freeze) {
  pid_freeze_integral(&pid_roll, freeze);
  pid_freeze_integral(&pid_pitch, freeze);
  pid_freeze_integral(&pid_yaw, freeze);
}
