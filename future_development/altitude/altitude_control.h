/**
 * @file altitude_control.h
 * @brief Altitude Hold Controller using PID
 *
 * HOW IT WORKS:
 * =============
 * 1. This module uses a PID controller to maintain a target altitude.
 * 2. Input: Target altitude (meters) and Current altitude (meters)
 * 3. Output: Throttle adjustment value (added to base hover throttle)
 *
 * CONTROL LOOP:
 * =============
 *   Target Altitude ─┐
 *                    ├─→ [PID Controller] ─→ Throttle Adjustment
 *   Current Altitude ┘                           ↓
 *                                        Base Throttle + Adjustment
 *                                                ↓
 *                                           Motor Mixer
 *
 * USAGE:
 * ======
 *   altitude_control_init();                    // Call once at startup
 *   altitude_control_set_target(1.5f);          // Hold at 1.5 meters
 *   float throttle_adj = altitude_control_update(current_alt, dt);
 *   uint16_t final_throttle = BASE_HOVER_THROTTLE + (int)throttle_adj;
 */

#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// CONFIGURATION - Tune these values for your quadcopter!
// ============================================================================

// PID Gains for Altitude Control
// DERIVED FROM CONTROL SYSTEM ANALYSIS (V2):
//   - Plant Model: G(s) = 0.0136 / (s² + 0.5s)
//   - Damping Ratio: ζ = 0.98
//   - All poles in LHP → STABLE ✓
//
// ANALYSIS RESULTS:
//   - Phase Margin: 85.1° (Excellent)
//   - Rise Time: 1.84 s
//   - Settling Time: 2.87 s
//   - Overshoot: 0.96% ✓ (< 3% target)
//
// YOUR SYSTEM: 880g, 1400KV, F450 - High mechanical gain!

#define ALT_KP                                                                 \
  49.0f // Proportional: Analysis optimal = 48.9
        // Range: 29 - 64
        // Increase if too slow, decrease if oscillating

#define ALT_KI                                                                 \
  2.5f // Integral: Analysis optimal = 2.4
       // Range: 1 - 4
       // Increase if altitude drifts, decrease if overshoots

#define ALT_KD                                                                 \
  80.0f // Derivative: Analysis optimal = 80.7
        // Range: 56 - 105
        // Critical for dampening - increase if bouncy

// Output Limits (in throttle units: 1000-2000)
#define ALT_OUTPUT_LIMIT 150.0f  // Max throttle adjustment (+/- 150)
#define ALT_INTEGRAL_LIMIT 80.0f // Prevent integral windup

// Base hover throttle - the throttle needed to hover at constant altitude
// You need to find this experimentally for YOUR quad
// Start around 1350-1450 for most setups
#define ALT_BASE_HOVER_THROTTLE 1400

// ============================================================================
// FUNCTIONS
// ============================================================================

/**
 * @brief Initialize the altitude controller
 * Call this once at startup, before using other functions
 */
void altitude_control_init(void);

/**
 * @brief Set the target altitude to hold
 * @param target_m Target altitude in meters (relative to ground)
 */
void altitude_control_set_target(float target_m);

/**
 * @brief Get current target altitude
 * @return Target altitude in meters
 */
float altitude_control_get_target(void);

/**
 * @brief Update the altitude controller (call this every control loop cycle)
 * @param current_alt_m Current altitude in meters (from baro/laser fusion)
 * @param dt_sec Time since last update in seconds (e.g., 0.004 for 250Hz)
 * @return Throttle adjustment value (can be positive or negative)
 *
 * EXAMPLE USAGE:
 *   float adjustment = altitude_control_update(fused_alt_m, 0.04f);
 *   uint16_t throttle = ALT_BASE_HOVER_THROTTLE + (int16_t)adjustment;
 */
float altitude_control_update(float current_alt_m, float dt_sec);

/**
 * @brief Enable or disable altitude hold mode
 * When disabled, the controller resets its state (clears integral, etc.)
 * @param enable true to enable, false to disable
 */
void altitude_control_enable(bool enable);

/**
 * @brief Check if altitude hold is currently enabled
 * @return true if enabled, false otherwise
 */
bool altitude_control_is_enabled(void);

/**
 * @brief Get the current altitude error (for debugging/blackbox)
 * @return Error in meters (target - current)
 */
float altitude_control_get_error(void);

/**
 * @brief Get the current integral term (for debugging)
 * @return Integral accumulator value
 */
float altitude_control_get_integral(void);

#endif // ALTITUDE_CONTROL_H
