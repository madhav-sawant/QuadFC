# Fix Log: Sensor Integration Conflict (VL53L0X & MPU6050)

**Date**: 2026-01-16
**Issue**: VL53L0X Laser Sensor returned "OUT OF RANGE" (or 0mm) errors when MPU6050 IMU was initialized on the same I2C bus.

## 1. Problem Description
- **Symptoms**:
  - VL53L0X works perfectly when MPU6050 is commented out.
  - VL53L0X works perfectly when BMP280 (Baro) is present.
  - **Conflict**: As soon as `imu_init()` is called, VL53L0X starts failing, despite I2C scanner showing all devices (0x29, 0x68, 0x76) present.
- **Hypothesis**:
  - Bus capacitance issues (too many pull-ups).
  - I2C Address conflict (Ruled out by scanner).
  - MPU6050 interfering with bus traffic (I2C Master Mode).
  - VL53L0X "Continuous Mode" getting desynchronized by bus noise.

## 2. Root Cause Analysis
1.  **MPU6050 I2C Master Mode**: The MPU6050 default configuration (or lack of reset) allowed it to occasionally act as an I2C Master, potentially hijacking the bus lines or confusing the VL53L0X.
2.  **VL53L0X Continuous Mode**: The "Continuous Back-to-Back" mode of the VL53L0X is sensitive to I2C timing disruptions. When MPU traffic was added, the VL53L0X internal state machine likely got stuck or desynchronized.
3.  **Electrical Environment**: Multiple modules with pull-up resistors created a "stiff" bus, making 400kHz communication marginal.

## 3. Solution Implemented

### A. I2C Bus Configuration
- **Speed Reduced**: Lowered I2C clock from 400kHz to **50kHz**. This ensures maximum signal integrity and robustness against capacitance.
- **Internal Pull-ups Disabled**: Set `sda_pullup_en` and `scl_pullup_en` to `GPIO_PULLUP_DISABLE` in `vl53l0x_i2c_init`. We rely on the external pull-up resistors on the sensor breakout boards.

### B. MPU6050 Configuration (`lib/imu/imu.c`)
Explicitly disabled the MPU's ability to control the AUX I2C bus.
```c
// Disable I2C Master Mode (Bit 5 of USER_CTRL 0x6A)
write_register(0x6A, 0x00);
// Enable I2C Bypass Mode (Bit 1 of INT_PIN_CFG 0x37)
write_register(0x37, 0x02);
```

### C. VL53L0X Operation Mode (`src/main.c`)
Switched from **Continuous Mode** to **Single Shot Mode**.
- **Change**: Removed `vl53l0x_start_continuous`.
- **Loop**: Used `vl53l0x_read_single()` inside the main loop.
- **Benefit**: Each measurement is a complete, atomic transaction (Start -> Wait -> Read). This effectively "resets" the sensor logic every cycle, preventing it from getting stuck in an invalid state.

### D. Initialization Order
Updated `main.c` to initialize the most "robust" sensors first:
1.  **Barometer** (BMP280)
2.  **IMU** (MPU6050)
3.  **VL53L0X** (Last, to ensure bus is quiet/stable before its sensitive init)

## 4. Verification
- **I2C Scan**: 0x29 (Laser), 0x68 (IMU), 0x76 (Baro) all detected.
- **Data Stream**:
  - Laser: Valid distance (mm)
  - IMU: Valid Roll/Pitch (deg)
  - Baro: Valid Altitude (mm)
- **Status**: **RESOLVED**
