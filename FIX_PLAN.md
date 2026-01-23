# 🛠️ Flight Controller Fix Plan

## 1. The Strategy: "Simplify & Stabilize"

We have identified two main problems causing your crashes and drift:
1.  **Yaw Spin (Crash)**: Use of "integral windup" where the drone tries harder and harder to turn while sitting on the ground, then spins violently when it takes off.
2.  **Drift**: The drone doesn't know what "level" is because the logging was broken (reporting 0.0°) and the accelerometer trim is likely off.

---

## 2. Library Guide: What does what?

| Library | Purpose | Your Analogy |
| :--- | :--- | :--- |
| **`imu`** | **The Inner Ear**. Reads Gyro (rotation speed) and Accel (gravity). Calculates "Am I level?" (Roll/Pitch angles). | *Must be calibrated perfectly flat.* |
| **`angle_control`** | **The Pilot**. Looks at the angle error (e.g., "I'm tilted 10° right, but stick is center") and demands a correction rate (e.g., "Roll Left at 20 deg/s!"). | *Outer Loop. Responsible for Level Mode.* |
| **`rate_control`** | **The Muscle**. Takes the rate demand ("Roll Left 20 deg/s") and fires the motors to make it happen using PID. | *Inner Loop. Responsible for crisp movements.* |
| **`pid`** | **The Brain**. Calculates P, I, and D terms to drive the error to zero. | *Needs to be stopped (frozen) when on the ground!* |
| **`mixer`** | **The Translator**. Converts Roll/Pitch/Yaw commands into 4 Motor speeds. | *Needs to match your frame (X-configuration).* |
| **`blackbox`** | **The Black Box**. Records flight data for us to analyze. | *Was missing Angle data (showing 0.0).* |

---

## 3. The Fixes

### Fix A: Stop the Spin (Yaw I-Term Freeze)
**Problem:** When the drone is on the ground with motors idle, the PID controller sees small movements and thinks "I need to correct this!". The "I-Term" (Integral) builds up huge power over time. As soon as you throttle up, that stored power releases instantly, spinning the drone.
**Solution:** We will tell the PID controller: *"If Throttle is low, DO NOT LEARN (Freeze I-Term)."*

### Fix B: Fix the Drift (Logging & Trim)
**Problem:** The logs showed Roll/Pitch as `0.00` because the main code wasn't saving them to the log file.
**Solution:**
1.  Enable Angle Logging in `main.c`.
2.  Once logged, we can see the *real* angle the drone thinks it is in.
3.  If it drifts, we will adjust `PITCH_TRIM_DEG` and `ROLL_TRIM_DEG` in `imu.c`.

---

## 4. Next Steps

1.  **I will apply the code fixes now.**
2.  **You will flash the firmware.**
3.  **New Test Flight:**
    *   **Props OFF first** to verify motors behave correctly on ground.
    *   **Hover Test**: Take off gently. If it drifts, note the direction.
    *   **Send me the new Blackbox Log**.

Let's fix the code!
