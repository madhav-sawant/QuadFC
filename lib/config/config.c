/**
 * @file config.c
 * @brief System configuration with NVS persistence
 */

#include "config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

system_config_t sys_cfg;

#define NVS_NAMESPACE "pid_tune_final"

void config_load_defaults(void) {
  // SYSTEMATIC PID TUNING - PROFILE 17 "Golden Tune - DJI Stability"
  // 
  // FLIGHT ANALYSIS (Blackbox 5):
  // - Rate Loop: LOCKED. No oscillation, holds angle perfectly.
  // - Yaw: Strong hold, counters torque.
  // - Angle: Calculated P=3.5 for 70dps correction at max tilt.
  //   I=0.20 adds position hold "memory" to fight steady-state drift (M1).
  
  // ANTI-OSCILLATION TUNE for High Gain System
  // Lower P = less aggressive correction
  // Original Stable PID Values
  sys_cfg.roll_kp = 0.50f;
  sys_cfg.roll_ki = 0.15f;
  sys_cfg.roll_kd = 0.012f;

  sys_cfg.pitch_kp = 0.50f;
  sys_cfg.pitch_ki = 0.15f;
  sys_cfg.pitch_kd = 0.012f;

  sys_cfg.yaw_kp = 3.50f;   // Strong tail hold
  sys_cfg.yaw_ki = 0.80f;   // Anti-drift integrator
  sys_cfg.yaw_kd = 0.00f;

  sys_cfg.rate_output_limit = 400.0f; // INCREASED from 180 to 400 to prevent saturation 
  sys_cfg.rate_integral_limit = 60.0f;

  // ANGLE MODE - Calculated Performance
  sys_cfg.angle_kp =  3.50f; // 20deg tilt -> 70dps command. Snappy.
  sys_cfg.angle_ki =  0.20f; // Corrects M1/CG drift over time.
  sys_cfg.angle_max = 20.0f;

  // Safety
  sys_cfg.low_bat_threshold = 10500; 
}

void config_save_to_nvs(void) {
  nvs_handle_t h;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
    return;

  nvs_set_blob(h, "roll_kp", &sys_cfg.roll_kp, sizeof(float));
  nvs_set_blob(h, "roll_ki", &sys_cfg.roll_ki, sizeof(float));
  nvs_set_blob(h, "roll_kd", &sys_cfg.roll_kd, sizeof(float));
  nvs_set_blob(h, "pitch_kp", &sys_cfg.pitch_kp, sizeof(float));
  nvs_set_blob(h, "pitch_ki", &sys_cfg.pitch_ki, sizeof(float));
  nvs_set_blob(h, "pitch_kd", &sys_cfg.pitch_kd, sizeof(float));
  nvs_set_blob(h, "yaw_kp", &sys_cfg.yaw_kp, sizeof(float));
  nvs_set_blob(h, "yaw_ki", &sys_cfg.yaw_ki, sizeof(float));
  nvs_set_blob(h, "yaw_kd", &sys_cfg.yaw_kd, sizeof(float));
  nvs_set_blob(h, "angle_kp", &sys_cfg.angle_kp, sizeof(float));

  nvs_commit(h);
  nvs_close(h);
}

static bool load_float(nvs_handle_t h, const char *key, float *out) {
  size_t len = sizeof(float);
  float val;
  if (nvs_get_blob(h, key, &val, &len) != ESP_OK)
    return false;
  if (val < -1000.0f || val > 1000.0f)
    return false; // Sanity check
  *out = val;
  return true;
}

bool config_load_from_nvs(void) {
  nvs_handle_t h;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
    return false;

  load_float(h, "roll_kp", &sys_cfg.roll_kp);
  load_float(h, "roll_ki", &sys_cfg.roll_ki);
  load_float(h, "roll_kd", &sys_cfg.roll_kd);
  load_float(h, "pitch_kp", &sys_cfg.pitch_kp);
  load_float(h, "pitch_ki", &sys_cfg.pitch_ki);
  load_float(h, "pitch_kd", &sys_cfg.pitch_kd);
  load_float(h, "yaw_kp", &sys_cfg.yaw_kp);
  load_float(h, "yaw_ki", &sys_cfg.yaw_ki);
  load_float(h, "yaw_kd", &sys_cfg.yaw_kd);
  load_float(h, "angle_kp", &sys_cfg.angle_kp);

  nvs_close(h);
  return true;
}
