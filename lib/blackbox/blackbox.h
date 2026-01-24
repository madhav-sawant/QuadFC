#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

#define BLACKBOX_MAX_ENTRIES 2000
#define BLACKBOX_LOG_DIVIDER 3  // 250Hz / 3 = 83Hz logging rate

typedef struct __attribute__((packed)) {
  float angle_roll, angle_pitch;
  float gyro_x, gyro_y, gyro_z;
  float pid_roll, pid_pitch, pid_yaw;
  uint16_t motor[4];
  uint16_t rc_roll, rc_pitch, rc_yaw, rc_throttle;
  uint16_t battery_mv;
  uint32_t i2c_errors;
  float accel_z_raw;
} blackbox_entry_t;

void blackbox_init(void);
void blackbox_log(const blackbox_entry_t *entry);
void blackbox_clear(void);
uint16_t blackbox_get_count(void);
const blackbox_entry_t *blackbox_get_entry(uint16_t index);
void blackbox_start(void);
void blackbox_stop(void);

#endif // BLACKBOX_H

