# IMU Sensor Fusion

## Overview

The **imu_sensor_fusion** project aims to read accelerometer and gyroscope data from the **ICM-42688-P** 6-axis MEMS MotionTracking sensor, apply a **Kalman filter** for sensor fusion to estimate orientation, and visualize raw and fused data. The project is developed in **C++17** using **CMake**, targeting a **Raspberry Pi** platform with **I2C communication** for sensor interaction.

---

## Current Status

### Data Structures
- Defined `accel`, `gyro`, and `IMUData` structs in `src/imu/imu_data.hpp` to store:
  - 18-bit accelerometer data  
  - 19-bit gyroscope data  
  - Temperature  
  - Timestamp  

### Abstract Interface
- Implemented `IMUReader` abstract base class in `src/imu/imu_reader.hpp`.
- Includes pure virtual methods for:
  - Initialization  
  - Reading accelerometer, gyroscope, and temperature  
  - Configuring sensor ranges  

### I2C Reader Skeleton
- Added `I2CReader` class in `src/imu/i2c_reader.hpp`.
- Stub implementations in `i2c_reader.cpp`, derived from `IMUReader`.
- Includes helper methods for register access.
- Full I2C implementation is delayed due to limited time for reviewing the ICM-42688-P datasheet.

### Build System
- Configured `CMakeLists.txt` to:
  - Build a static library (`imu_reader`)
  - Build a test executable (`imu_app`)
  - Support future integration of the I2C library

### Version Control
- Added `.gitignore` to exclude build artifacts.

---

## Future Goals

- Continue stub implementation in `I2CReader` due to hardware unavailability and limited time for datasheet review.
- Implement `I2CReader` to communicate with the **ICM-42688-P** via I2C:
  - Configure �16g accelerometer range  
  - Configure �2000 dps gyroscope range  
  - Set 100 Hz output data rate  
- Develop a `SimulatedIMUReader` for testing without hardware.
- Design and implement a **Kalman filter** for sensor fusion.
- Add visualization of raw and fused data:
  - Using tools like **ImPlot** or **matplotlib-cpp**

---

## Build Instructions

### Clone the repository:
```bash
git clone https://github.com/your-username/imu_sensor_fusion.git
cd imu_sensor_fusion
