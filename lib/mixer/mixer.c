/**
 * @file mixer.c
 * @brief Quad-X motor mixer
 *
 * Motor layout (props-out):
 *   M4(CCW)──────M2(CW)     FRONT
 *       ╲      ╱
 *         ╲  ╱
 *         ╱  ╲
 *       ╱      ╲
 *   M3(CW)──────M1(CCW)     REAR
 */

#include "mixer.h"
#include "../pwm/pwm.h"

static bool armed = false;
static uint16_t motors[4] = {1000, 1000, 1000, 1000};

static uint16_t clamp(int32_t val) {
  if (val < MIXER_IDLE_THROTTLE)
    return MIXER_IDLE_THROTTLE;
  if (val > MIXER_MAX_THROTTLE)
    return MIXER_MAX_THROTTLE;
  return (uint16_t)val;
}

void mixer_init(void) {
  armed = false;
  for (int i = 0; i < 4; i++) {
    motors[i] = MIXER_STOP_CMD;
  }
}

// Yaw trim to compensate for CW/CCW motor thrust imbalance
// Positive value = bias toward CW motors (fights left spin)
// Adjust this value if quad still drifts: increase if spinning left, decrease
// if spinning right
static const float YAW_TRIM = 0.0f;  // Disabled - let PID I-term handle bias

void mixer_update(uint16_t throttle, float roll, float pitch, float yaw) {
  // Apply yaw trim to pre-compensate for mechanical bias
  yaw = yaw + YAW_TRIM;

  if (!armed) {
    for (int i = 0; i < 4; i++) {
      motors[i] = MIXER_STOP_CMD;
      pwm_set_motor(i, MIXER_STOP_CMD);
    }
    return;
  }

  // Idle spin without PID mixing at low throttle
  if (throttle < MIXER_IDLE_THROTTLE + 50) {
    for (int i = 0; i < 4; i++) {
      motors[i] = MIXER_IDLE_THROTTLE;
      pwm_set_motor(i, MIXER_IDLE_THROTTLE);
    }
    return;
  }

  int32_t t = throttle;
  if (t > MIXER_MAX_THROTTLE)
    t = MIXER_MAX_THROTTLE;

  // Quad-X mixing (pitch sign inverted for hardware adaptation)
  // YAW SIGN FIX: Positive yaw command should create CW torque (right rotation)
  // CCW motors (M1, M4): SUBTRACT yaw to slow down (reduce CCW torque)
  // CW motors (M2, M3): ADD yaw to speed up (increase CW torque)
  int32_t m1 = t - (int32_t)roll + (int32_t)pitch - (int32_t)yaw; // Rear Right CCW
  int32_t m2 = t - (int32_t)roll - (int32_t)pitch + (int32_t)yaw; // Front Right CW
  int32_t m3 = t + (int32_t)roll + (int32_t)pitch + (int32_t)yaw; // Rear Left CW
  int32_t m4 = t + (int32_t)roll - (int32_t)pitch - (int32_t)yaw; // Front Left CCW

  motors[0] = clamp(m1);
  motors[1] = clamp(m2);
  motors[2] = clamp(m3);
  motors[3] = clamp(m4);

  for (int i = 0; i < 4; i++) {
    pwm_set_motor(i, motors[i]);
  }
}

void mixer_arm(bool arm) {
  armed = arm;
  if (!arm) {
    mixer_update(1000, 0, 0, 0);
  }
}

void mixer_get_outputs(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4) {
  *m1 = motors[0];
  *m2 = motors[1];
  *m3 = motors[2];
  *m4 = motors[3];
}
