#ifndef PWM_H
#define PWM_H

#include <stdint.h>

#define PWM_FREQ_HZ 250 // Must match control loop frequency (250Hz)
#define PWM_RES_BIT 12
#define PWM_MOTOR_COUNT 4
#define PWM_MIN_PULSE_US 1000
#define PWM_MAX_PULSE_US 2000

#define PWM_MOTOR_1_GPIO 13
#define PWM_MOTOR_2_GPIO 27
#define PWM_MOTOR_3_GPIO 26
#define PWM_MOTOR_4_GPIO 25

void pwm_init(void);
void pwm_set_motor(int motor_index, uint32_t pulse_width_us);

#endif // PWM_H
