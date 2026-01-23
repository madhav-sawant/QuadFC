/**
 * @file altitude_control.c
 * @brief Altitude Hold Controller Implementation
 *
 * WHAT THIS FILE DOES:
 * ====================
 * Implements a PID controller for altitude hold.
 * Takes current altitude measurement and outputs throttle adjustment.
 *
 * KEY CONCEPTS:
 * =============
 * 1. ERROR = Target Altitude - Current Altitude
 *    - Positive error means we're too low → need more throttle
 *    - Negative error means we're too high → need less throttle
 *
 * 2. P-TERM (Proportional):
 *    - Immediate response to error
 *    - Kp * error
 *    - Higher Kp = faster response but can oscillate
 *
 * 3. I-TERM (Integral):
 *    - Accumulates error over time
 *    - Fixes "steady-state error" (when you're stuck 10cm below target)
 *    - Ki * sum(error * dt)
 *    - Problem: Can "wind up" if error persists too long
 *
 * 4. D-TERM (Derivative):
 *    - Predicts future error by looking at rate of change
 *    - Dampens oscillations
 *    - Kd * (d_measurement / dt)
 *    - We use "Derivative on Measurement" to avoid spikes when target changes
 */

#include "altitude_control.h"
#include <math.h>
#include <stdio.h>

// ============================================================================
// INTERNAL STATE
// ============================================================================

// Why 'static'?
// - These variables keep their value between function calls
// - They are only visible inside this file (encapsulation)

static float target_altitude_m = 0.0f;   // Target altitude to hold
static float altitude_error = 0.0f;      // Current error (for debugging)
static float integral = 0.0f;            // I-term accumulator
static float prev_altitude = 0.0f;       // Previous altitude (for D-term)
static float filtered_derivative = 0.0f; // Low-pass filtered D-term
static bool is_enabled = false;          // Altitude hold on/off

// D-term filter coefficient
// Lower value = more filtering = smoother but slower
// Higher value = less filtering = faster but noisier
#define D_TERM_LPF_ALPHA 0.2f

// ============================================================================
// INITIALIZATION
// ============================================================================

void altitude_control_init(void) {
  // Reset all state variables to zero
  // This ensures clean startup every time

  target_altitude_m = 0.0f;
  altitude_error = 0.0f;
  integral = 0.0f;
  prev_altitude = 0.0f;
  filtered_derivative = 0.0f;
  is_enabled = false;

  printf("ALTITUDE: Controller initialized\n");
}

// ============================================================================
// SETTERS & GETTERS
// ============================================================================

void altitude_control_set_target(float target_m) {
  target_altitude_m = target_m;
  printf("ALTITUDE: Target set to %.2f m\n", target_m);
}

float altitude_control_get_target(void) { return target_altitude_m; }

void altitude_control_enable(bool enable) {
  if (enable && !is_enabled) {
    // Enabling: Reset state to prevent jumps
    integral = 0.0f;
    filtered_derivative = 0.0f;
    printf("ALTITUDE: Hold ENABLED at target %.2f m\n", target_altitude_m);
  } else if (!enable && is_enabled) {
    printf("ALTITUDE: Hold DISABLED\n");
  }
  is_enabled = enable;
}

bool altitude_control_is_enabled(void) { return is_enabled; }

float altitude_control_get_error(void) { return altitude_error; }

float altitude_control_get_integral(void) { return integral; }

// ============================================================================
// MAIN PID CALCULATION
// ============================================================================

/**
 * This is called every control loop cycle (e.g., 250 Hz = every 4ms)
 *
 * STEP BY STEP:
 * 1. Calculate error (how far from target)
 * 2. Calculate P-term (immediate correction)
 * 3. Calculate I-term (accumulated correction)
 * 4. Calculate D-term (rate-of-change dampening)
 * 5. Sum them up and clamp to limits
 */
float altitude_control_update(float current_alt_m, float dt_sec) {
  // If disabled, return 0 (no throttle adjustment)
  if (!is_enabled) {
    return 0.0f;
  }

  // Prevent division by zero
  if (dt_sec <= 0.0f) {
    dt_sec = 0.004f; // Default to 250Hz
  }

  // -------------------------------------------------------------------------
  // STEP 1: Calculate Error
  // -------------------------------------------------------------------------
  // Error = Where we want to be - Where we are
  // Positive error = we're too low, need to go UP (more throttle)
  // Negative error = we're too high, need to go DOWN (less throttle)

  altitude_error = target_altitude_m - current_alt_m;

  // -------------------------------------------------------------------------
  // STEP 2: P-TERM (Proportional)
  // -------------------------------------------------------------------------
  // Proportional response: Big error = Big correction
  // Example: If Kp=150 and error=0.1m (10cm low)
  //          P_output = 150 * 0.1 = 15 throttle units

  float p_out = ALT_KP * altitude_error;

  // -------------------------------------------------------------------------
  // STEP 3: I-TERM (Integral)
  // -------------------------------------------------------------------------
  // Integral accumulates error over time
  // Formula: integral = integral + (error * dt)
  //
  // WHY WE NEED THIS:
  // - Imagine your quad hovers 5cm too low
  // - P-term alone might not be strong enough to fix it
  // - I-term slowly builds up until it provides enough correction
  //
  // ANTI-WINDUP:
  // - We limit the integral to prevent it from growing too large
  // - Otherwise, if you hold the quad down, integral would grow huge
  //   and when released, it would rocket upward

  integral += altitude_error * dt_sec;

  // Clamp integral to limits (anti-windup)
  if (integral > ALT_INTEGRAL_LIMIT) {
    integral = ALT_INTEGRAL_LIMIT;
  } else if (integral < -ALT_INTEGRAL_LIMIT) {
    integral = -ALT_INTEGRAL_LIMIT;
  }

  float i_out = ALT_KI * integral;

  // -------------------------------------------------------------------------
  // STEP 4: D-TERM (Derivative)
  // -------------------------------------------------------------------------
  // Derivative looks at how FAST the altitude is changing
  // If altitude is rising quickly, D-term reduces throttle to slow it down
  // If altitude is falling quickly, D-term adds throttle to slow the fall
  //
  // DERIVATIVE ON MEASUREMENT (not error):
  // - We calculate d(altitude)/dt, not d(error)/dt
  // - This prevents "derivative kick" when target changes suddenly
  //
  // WHY FILTER THE D-TERM:
  // - Derivative amplifies noise (small sensor fluctuations become big)
  // - Low-pass filter smooths this out
  // - Formula: filtered = alpha * new + (1-alpha) * old

  float raw_derivative = -(current_alt_m - prev_altitude) / dt_sec;
  // Note: Negative sign because we want to oppose the change
  // If altitude increases, we want negative D-output to slow it down

  // Low-pass filter the derivative
  filtered_derivative = D_TERM_LPF_ALPHA * raw_derivative +
                        (1.0f - D_TERM_LPF_ALPHA) * filtered_derivative;

  float d_out = ALT_KD * filtered_derivative;

  // Save current altitude for next iteration
  prev_altitude = current_alt_m;

  // -------------------------------------------------------------------------
  // STEP 5: Sum and Clamp Output
  // -------------------------------------------------------------------------
  float output = p_out + i_out + d_out;

  // Clamp to output limits
  // This prevents the controller from commanding impossible throttle values
  if (output > ALT_OUTPUT_LIMIT) {
    output = ALT_OUTPUT_LIMIT;
  } else if (output < -ALT_OUTPUT_LIMIT) {
    output = -ALT_OUTPUT_LIMIT;
  }

  return output;
}
