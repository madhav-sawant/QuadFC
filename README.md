# 🚁 ESP32 Quadcopter Flight Controller
**Final Year Engineering Project**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://espressif.com)

A high-performance, DIY flight controller software built from scratch for the ESP32. Designed for F450 class quadcopters, featuring cascaded PID control, self-leveling, and safety features.

---

## 🛠️ Hardware Specifications

| Component | Recommendation | Your Control Rig |
|-----------|----------------|------------------|
| **Microcontroller** | ESP32-WROOM-32 | Core 1: Control Loop (250Hz), Core 0: Housekeeping |
| **IMU** | MPU6050 (I2C) | Gyro + Accelerometer @ 400kHz |
| **Frame** | F450 / F330 | **F450** (Glass Fiber / Nylon) |
| **Motors** | 2212 920KV-1400KV | **1400KV** Brushless Outrunners |
| **Propellers** | 8045 / 1045 | **8045** (8-inch, 4.5 pitch) |
| **ESC** | 30A BLHeli/SimonK | Standard PWM (1000-2000µs) |
| **Receiver** | IBUS Compatible | FlySky i6 / Turnigy |
| **Battery** | 3S LiPo (2200mAh+) | Voltage monitored via voltage divider |

---

## 🧠 System Architecture

The software uses a **Cascaded Control Loop** architecture running at **250Hz (4ms cycle)**.

### 1. Control Loops
*   **Outer Loop (Angle Mode):**
    *   **Input:** Pilot Stick Angle (Target) vs. IMU Angle (Actual)
    *   **Controller:** PI (Proportional-Integral)
    *   **Output:** Target Rotation Rate (deg/sec)
    *   *Note: This provides the "Self-Leveling" capability.*

*   **Inner Loop (Rate Mode):**
    *   **Input:** Target Rate (from Outer Loop) vs. Gyro Rate (Actual)
    *   **Controller:** PID (Proportional-Integral-Derivative)
    *   **Output:** Motor Corrections
    *   *Note: This provides the stabilization throughout the flight.*

### 2. Sensor Fusion (IMU)
*   **Complementary Filter:** Fuses noisy accelerometer data (gravity reference) with drifting gyro data.
*   **Alpha = 0.996:** Highly trusted gyro for smooth flight, with strong enough accelerometer weight (0.4%) to correct drift quickly (~1 sec correction time).
*   **Calibration:** Accelerometer offset is calibrated **ONCE** and saved to NVS (flash memory). Gyro calibrates at every boot.

### 3. Mixer (Quad-X)
Standard X-configuration mixing matrix:
*   **Motor 1 (Rear Right, CCW):** `Throttle - Roll + Pitch - Yaw`
*   **Motor 2 (Front Right, CW):**  `Throttle - Roll - Pitch + Yaw`
*   **Motor 3 (Rear Left, CW):**    `Throttle + Roll + Pitch + Yaw`
*   **Motor 4 (Front Left, CCW):**  `Throttle + Roll - Pitch - Yaw`

---

## 🚀 How to Use / Flight Manual

### 1. Initial Setup (One-Time Calibration)
**Crucial Step:** The accelerometer determines what "level" is.
1.  Place the drone on a **perfectly level surface**.
2.  Press and **HOLD the BOOT button** on the ESP32.
3.  Power on the drone/connect USB while holding the button.
4.  Wait for the LED to turn ON, then release.
5.  Calibration is performed and **saved permanently** to flash memory.
6.  The LED will blink 3 times to confirm success.

### 2. Normal Startup (Every Flight)
1.  Place drone on ground (doesn't need to be perfectly level, just still).
2.  Power on.
3.  **DO NOT TOUCH** the drone for 2 seconds while the Gyro calibrates.
4.  **Level Check:**
    *   **2 Quick Blinks:** IMU is level and ready.
    *   **5 Slow Blinks:** Drone is tilted >2 degrees. Take off with caution or re-level.

### 3. Arming & Flying
*   **Safety:** The drone will NOT arm if the throttle is not at zero.
*   **Input:** Currently configured for **IBUS** receiver (FlySky standard).
*   **To ARM:**
    1.  Lower Throttle to zero.
    2.  Flip **Channel 5 (AUX1)** switch HIGH (>1600us).
    3.  Check serial monitor/LED: "ARMED".
*   **To DISARM:** Flip Channel 5 switch LOW.

### 4. Emergency Stop
*   **Software Kill Switch:** Disarm via TX switch.
*   **Hardware Panic Button:** Pressing the **BOOT button** while flying will immediately kill all motors (Emergency Stop). Use only in case of crashes or flyaways!

---

## 💻 Code Structure (`/lib`)

The codebase is modular, mimicking professional flight stacks like Betaflight/PX4.

*   `main.c`: The conductor. Setup, scheduling, and the main 250Hz loop.
*   `imu/`: MPU6050 driver, calibration logic, and complementary filter.
*   `pid/`: Generic PID controller math with D-term filtering and anti-windup.
*   `angle_control/`: Outer loop logic (converts Angle Error -> Rate Target).
*   `rate_control/`: Inner loop logic (converts Rate Error -> Motor Mix).
*   `mixer/`: Mathematical mixing for Quad-X configuration.
*   `pwm/`: ESP32 LEDC (PWM) driver for ESCs (400Hz update rate).
*   `rx/`: IBUS receiver driver (UART based).
*   `config/`: Handling of settings (PID gains) and persistence (NVS).
*   `blackbox/`: Data logging system (stores flight data to internal flash).
*   `adc/`: Battery voltage monitoring.

---

## 🔧 Tuning Guide (Current Values)

The current tune is set for a "High Gain" system (F450/1400KV).

**Rate Loop (Inner)**
*   **Roll/Pitch:** P=0.45, I=0.15, D=0.012
    *   *Locked in. High P provides snap, I corrects imbalance.*
*   **Yaw:** P=3.50, I=0.80
    *   *Very stiff yaw to counter torque from large 8" props.*

**Angle Loop (Outer)**
*   **Level Strength (P):** 3.50 (Strong self-leveling)
*   **Drift Correction (I):** 0.20
    *   *Corrects for Center of Gravity (CG) imbalances over time.*

---

## ⚡ Troubleshooting

**1. Drone Drifts Forward/Backward**
*   **Cause:** Accelerometer calibration offset.
*   **Fix:** Perform the **One-Time Calibration** (see above) on a truly level surface. Use a bubble level!

**2. Drifts in Yaw (Spins)**
*   **Cause:** Mechanical twist in motors or compass/gyro drift.
*   **Fix:** The high I-gain on Yaw (0.80) usually fixes this. Ensure props are tight.

**3. Wobbles Fast (Oscillation)**
*   **Cause:** P-gain too high.
*   **Fix:** Reduce Rate P-gain slightly (0.45 -> 0.40).

**4. Won't Arm**
*   **Cause:** Safety checks.
*   **Fix:** Ensure Throttle is at minimum. Ensure drone is not upside down (Crash protection). Check RX connection.

---

## 📝 Credits
**Author:** 4th Year Engineering Student  
**Project:** Quadcopter Flight Controller Implementation
**Status:** Flight Ready v1.0

*"It flies, it crashes, it learns. Just like its engineer."*
