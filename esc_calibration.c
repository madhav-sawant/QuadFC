/**
 * ESC CALIBRATION + INTERACTIVE MOTOR TEST
 * =========================================
 * After calibration, enter motor number (1-4) to test, or 5 for all motors.
 */

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

// Motor GPIOs
#define MOTOR_1_GPIO 13 // Rear Right
#define MOTOR_2_GPIO 27 // Front Right
#define MOTOR_3_GPIO 26 // Rear Left
#define MOTOR_4_GPIO 25 // Front Left

#define PWM_FREQ_HZ 250 // Matches flight code (pwm.h)
#define PWM_MAX_US 2000
#define PWM_MIN_US 1000
#define PWM_TEST_US 1200 // Safe test throttle

static int motor_gpios[] = {MOTOR_1_GPIO, MOTOR_2_GPIO, MOTOR_3_GPIO,
                            MOTOR_4_GPIO};
static const char *motor_names[] = {"M1 (Rear Right)", "M2 (Front Right)",
                                    "M3 (Rear Left)", "M4 (Front Left)"};

static uint32_t us_to_duty(uint32_t us) {
  // duty = (us * resolution * frequency) / 1,000,000
  return (us * 8192 * PWM_FREQ_HZ) / 1000000;
}

static void init_pwm(void) {
  ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                               .duty_resolution = LEDC_TIMER_13_BIT,
                               .timer_num = LEDC_TIMER_0,
                               .freq_hz = PWM_FREQ_HZ,
                               .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&timer);

  for (int i = 0; i < 4; i++) {
    ledc_channel_config_t ch = {.gpio_num = motor_gpios[i],
                                .speed_mode = LEDC_LOW_SPEED_MODE,
                                .channel = (ledc_channel_t)i,
                                .timer_sel = LEDC_TIMER_0,
                                .duty = 0,
                                .hpoint = 0};
    ledc_channel_config(&ch);
  }
}

static void set_motor(int idx, uint32_t us) {
  uint32_t duty = us_to_duty(us);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
}

static void set_all_motors(uint32_t us) {
  for (int i = 0; i < 4; i++)
    set_motor(i, us);
}

static void test_motor(int m) {
  printf(">>> TESTING: %s <<<\n", motor_names[m]);
  set_all_motors(PWM_MIN_US);
  vTaskDelay(pdMS_TO_TICKS(300));
  set_motor(m, PWM_TEST_US);
  printf("    SPINNING for 3 seconds...\n");
  vTaskDelay(pdMS_TO_TICKS(3000));
  set_motor(m, PWM_MIN_US);
  printf("    STOPPED\n\n");
}

static void test_all_motors(void) {
  printf(">>> ALL 4 MOTORS SPINNING <<<\n");
  set_all_motors(PWM_TEST_US);
  vTaskDelay(pdMS_TO_TICKS(5000));
  set_all_motors(PWM_MIN_US);
  printf("    STOPPED\n\n");
}

void app_main(void) {
  printf("\n\n");
  printf("================================================\n");
  printf("   ESC CALIBRATION + MOTOR TEST\n");
  printf("================================================\n\n");

  init_pwm();

  // ========== CALIBRATION ==========
  printf(">>> MAKE SURE BATTERY IS UNPLUGGED! <<<\n\n");
  printf("Waiting 5 seconds...\n");
  for (int i = 5; i > 0; i--) {
    printf("  %d...\n", i);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  printf("\n>>> SETTING THROTTLE TO MAX (2000us) <<<\n");
  set_all_motors(PWM_MAX_US);

  printf("\n");
  printf("╔══════════════════════════════════════════════╗\n");
  printf("║     >>> PLUG IN BATTERY NOW! <<<             ║\n");
  printf("║                                              ║\n");
  printf("║  Wait for ALL 4 ESCs to beep HIGH tones      ║\n");
  printf("╚══════════════════════════════════════════════╝\n\n");

  printf("Waiting 10 seconds for you to plug battery...\n");
  for (int i = 10; i > 0; i--) {
    printf("  %d...\n", i);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  printf("\n>>> SETTING THROTTLE TO MIN (1000us) <<<\n");
  printf(">>> ALL 4 ESCs should beep LOW tones NOW! <<<\n\n");
  set_all_motors(PWM_MIN_US);

  vTaskDelay(pdMS_TO_TICKS(3000));

  printf("╔══════════════════════════════════════════════╗\n");
  printf("║        CALIBRATION COMPLETE!                 ║\n");
  printf("╚══════════════════════════════════════════════╝\n\n");

  // ========== AUTO TEST ==========
  printf(">>> AUTO MOTOR TEST <<<\n\n");
  for (int m = 0; m < 4; m++) {
    test_motor(m);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  test_all_motors();

  // ========== INTERACTIVE MODE ==========
  printf("\n");
  printf("================================================\n");
  printf("         INTERACTIVE MOTOR TEST MODE\n");
  printf("================================================\n");
  printf("\n");
  printf("Type a number and press ENTER:\n");
  printf("  1 = Test M1 (Rear Right)\n");
  printf("  2 = Test M2 (Front Right)\n");
  printf("  3 = Test M3 (Rear Left)\n");
  printf("  4 = Test M4 (Front Left)\n");
  printf("  5 = Test ALL motors together\n");
  printf("  0 = Stop all motors\n");
  printf("\n");

  while (1) {
    int c = getchar();
    if (c != EOF) {
      switch (c) {
      case '1':
        test_motor(0);
        break;
      case '2':
        test_motor(1);
        break;
      case '3':
        test_motor(2);
        break;
      case '4':
        test_motor(3);
        break;
      case '5':
        test_all_motors();
        break;
      case '0':
        set_all_motors(PWM_MIN_US);
        printf("All motors STOPPED\n\n");
        break;
      default:
        // Ignore other chars (like newlines)
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
