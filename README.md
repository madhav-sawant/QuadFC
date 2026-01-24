# QuadFC - ESP32 Based Flight Controller

A custom flight controller firmware built from scratch on ESP32. Inspired by Betaflight and ArduPilot, but designed for learning and experimentation.

**Status:** Angle mode tuned and working. Future updates in progress.

---

## Demo

![Drone Demo](assets/QuadFc.gif)

---

## About

This project started as a way to understand how drones actually work under the hood. Instead of using off-the-shelf flight controllers, I decided to build the firmware myself on an ESP32.

The goal was to replicate what Betaflight and ArduPilot do, but at a smaller and more accessible level. This isn't meant for commercial use - it's purely for learning, experimenting, and tinkering.

---

## Important Note on PID Values

This setup has **high mechanical gain** because of the 1400KV motors with 8045 props. That's why the PID values in this firmware are relatively low.

If you watch YouTube tutorials (like Joop Brokking or Carbon Aeronautics) where people use much higher PID values, it's because their mechanical gain is different (smaller props, lower KV motors, etc).

If you want easier tuning with lower mechanical gain:
- Use 1000KV motors
- Use 8045 props (10-inch)
- Same F450 frame

Also note: This firmware is written in **pure C** using the ESP-IDF framework, not Arduino C++.


---

## Features

- Cascaded PID control (outer angle loop + inner rate loop) running at 250Hz
- Self-leveling angle mode with automatic horizon correction
- Complementary filter for sensor fusion (gyro + accelerometer)
- One-time accelerometer calibration saved to flash memory
- Crash detection - auto disarms if the drone tilts too far or spins out of control
- RX failsafe - motors shut off if signal is lost
- Emergency stop button (BOOT button kills motors instantly)
- Low battery warning with LED indication
- PPM receiver input - works with FlySky and similar transmitters

---

## Hardware Used

| Component | What I Used |
|-----------|-------------|
| Flight Controller | ESP32-WROOM-32 |
| IMU | MPU6050 |
| Frame | F450 |
| Motors | 1400KV Brushless |
| Propellers | 8045 |
| ESCs | 30A |
| Battery | 3S 2200mAh LiPo |
| Transmitter | FlySky FS-i6 |
| Receiver | FlySky FS-iA6B (PPM mode) |

---

## Wiring

```
ESP32 Pin    ->    Component
-----------------------------------
GPIO 13      ->    Motor 1 (Rear Right)
GPIO 27      ->    Motor 2 (Front Right)
GPIO 26      ->    Motor 3 (Rear Left)
GPIO 25      ->    Motor 4 (Front Left)
GPIO 33      ->    Receiver PPM Signal
GPIO 34      ->    Battery Voltage (via divider)
GPIO 21      ->    MPU6050 SDA
GPIO 22      ->    MPU6050 SCL
GPIO 2       ->    Status LED
GPIO 0       ->    BOOT Button
```

## Motor Layout

```
Motor positions (Quad-X, props-out):

   M4(CCW)──────M2(CW)      FRONT
       ╲      ╱
         ╲  ╱
         ╱  ╲
       ╱      ╲
   M3(CW)──────M1(CCW)      REAR

CCW = Counter-Clockwise
CW  = Clockwise
```

---

## How to Build and Flash

You'll need PlatformIO installed (VS Code extension works great).

```bash
# Clone the repo
git clone https://github.com/madhav-sawant/QuadFC.git
cd QuadFC

# Build and upload
pio run --target upload

# Open serial monitor
pio device monitor
```

---

## Calibration

Before first flight, you need to calibrate the accelerometer:

1. Place the drone on a flat, level surface
2. Hold the BOOT button while powering on
3. Keep holding until the LED starts blinking
4. Release the button and wait for calibration to complete
5. Done - calibration is saved permanently

---

## How to Arm and Fly

1. Power on the drone and keep it still (gyro calibrates automatically)
2. Make sure throttle is at minimum
3. Flip the arm switch (Channel 5) to HIGH
4. LED turns solid - drone is armed
5. Slowly raise throttle and fly

To disarm, flip the arm switch back to LOW.

---

## System Architecture

```
                    CONTROL LOOP (250Hz)

    IMU ──────> ANGLE LOOP ──────> RATE LOOP ──────> MIXER
   (MPU6050)     (PI)              (PID)            (Quad-X)
                   ^                                   |
                   |                                   v
    RECEIVER ──────┘                              M1  M2  M3  M4
     (PPM)
```

The system uses a cascaded control structure:
- Outer loop (Angle) - maintains the desired tilt angle
- Inner loop (Rate) - handles fast stabilization using gyro data

---

## Safety Features

| What | When it triggers | What happens |
|------|------------------|--------------|
| Crash detection | Angle > 60° or Gyro > 2000°/s | Motors shut off |
| RX failsafe | No signal for 200ms | Motors shut off |
| Emergency stop | BOOT button pressed | Motors shut off immediately |
| Arm safety | Throttle not at zero | Won't arm |
| Low battery | Voltage < 10.5V | LED blinks warning |

---

## What's Next

Currently working on:
- Barometer-based altitude hold

Planned for future:
- GPS position hold
- Waypoint navigation
- Return to home

---

## Project Structure

```
QuadFC/
├── src/
│   └── main.c              # Main flight controller
├── lib/
│   ├── adc/                # Battery monitoring
│   ├── angle_control/      # Outer loop
│   ├── config/             # Settings & storage
│   ├── imu/                # MPU6050 & sensor fusion
│   ├── mixer/              # Motor mixing
│   ├── pid/                # PID math
│   ├── pwm/                # ESC output
│   ├── rate_control/       # Inner loop
│   └── rx/                 # Receiver driver
├── platformio.ini
└── README.md
```

---

## Credits

**Firmware:** Madhav Sawant

**Inspired by:**
- Betaflight
- ArduPilot
- ESP-Drone by Espressif


---

## License

This project is for **educational and personal use only**.

You are free to:
- Learn from this code
- Modify it for personal projects
- Share it with proper credit

You are NOT allowed to:
- Use this in any commercial product
- Sell this code or products based on it

