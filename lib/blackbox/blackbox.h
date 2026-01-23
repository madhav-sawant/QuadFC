#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

// Buffer configuration
// Increased to 3000 entries (approx 36s at 83Hz)
#define BLACKBOX_MAX_ENTRIES 2000

// Log rate: 250Hz / 3 = 83Hz
#define BLACKBOX_LOG_DIVIDER 3

// ============================================================================
// SIMPLIFIED LOG ENTRY STRUCTURE (40 bytes)
// ============================================================================
typedef struct __attribute__((packed)) {
  // Angle Data (for drift analysis)
  float angle_roll, angle_pitch; // 8

  // Tuning Data
  float gyro_x, gyro_y, gyro_z;       // 12
  float pid_roll, pid_pitch, pid_yaw; // 12

  // Motor Output (Saturation check)
  uint16_t motor[4]; // 8

  // RC Input (Reference)
  uint16_t rc_roll;     // 2
  uint16_t rc_pitch;    // 2
  uint16_t rc_yaw;      // 2
  uint16_t rc_throttle; // 2

  // Health
  uint16_t battery_mv; // 2
  uint32_t i2c_errors; // 4 - Incremental I2C I/O error count
  float accel_z_raw;   // 4 - Filtered Accel Z (diagnostic for vibration)

} blackbox_entry_t;

// ============================================================================
// FLAG BIT DEFINITIONS (expanded)
// ============================================================================
#define BLACKBOX_FLAG_ARMED (1 << 0)       // Motors armed
#define BLACKBOX_FLAG_ERROR (1 << 1)       // System error
#define BLACKBOX_FLAG_CRASH_ANGLE (1 << 2) // Disarm: Angle > 60 deg
#define BLACKBOX_FLAG_CRASH_GYRO (1 << 3)  // Disarm: Gyro > 2000 dps
#define BLACKBOX_FLAG_LOW_BAT (1 << 4)     // Low battery warning
#define BLACKBOX_FLAG_RX_LOSS (1 << 5)     // RC signal lost
#define BLACKBOX_FLAG_GYRO_SAT (1 << 6)    // Gyro saturation detected
#define BLACKBOX_FLAG_PID_SAT (1 << 7)     // PID output saturated
#define BLACKBOX_FLAG_ESTOP (1 << 8)       // Emergency stop button pressed

// API Functions

/**
 * @brief Initialize the blackbox system.
 *        Creates a FreeRTOS task on Core 0 for processing log entries.
 */
void blackbox_init(void);

/**
 * @brief Log a flight data entry (non-blocking).
 *        Safe to call from ISR or control loop.
 *        If queue is full, entry is dropped (no blocking).
 * @param entry Pointer to the entry data to log.
 */
void blackbox_log(const blackbox_entry_t *entry);

/**
 * @brief Clear all logged entries.
 */
void blackbox_clear(void);

/**
 * @brief Get the number of logged entries.
 * @return Number of entries in the buffer.
 */
uint16_t blackbox_get_count(void);

/**
 * @brief Get a logged entry by index.
 * @param index Index of the entry (0 to count-1).
 * @return Pointer to the entry, or NULL if index out of range.
 */
const blackbox_entry_t *blackbox_get_entry(uint16_t index);

/**
 * @brief Start recording (enabled by default after init).
 */
void blackbox_start(void);

/**
 * @brief Stop recording.
 */
void blackbox_stop(void);

#endif // BLACKBOX_H
