# IMU Sensor Fusion

## Overview

The **imu_sensor_fusion** project aims to read accelerometer and gyroscope data from the **ICM-42688-P** 6-axis MEMS MotionTracking sensor, apply a **Kalman filter** for sensor fusion to estimate orientation, and visualize raw and fused data. The project is developed in **C++17** using **CMake**, targeting a **Raspberry Pi** platform with **I2C communication** for sensor interaction.

---

## Current Status

- **Data Structures**: Defined `accel`, `gyro`, and `IMUData` structs in `src/imu/imu_data.hpp` to store raw sensor data (18-bit accelerometer, 19-bit gyroscope, temperature, timestamp).

- **Abstract Interface**: Implemented `IMUReader` abstract base class in `src/imu/imu_reader.hpp` with pure virtual methods for initialization, reading accelerometer/gyroscope/temperature, and configuring sensor ranges.

- **I2C Reader Skeleton**: Added `I2CReader` class in `src/imu/i2c_reader.hpp`, derived from `IMUReader`, with stubs for I2C communication and helper methods for register access. Implementation is pending datasheet review.

- **Build System**: Configured `CMakeLists.txt` to build a static library (`imu_reader`) and test executable (`imu_app`), with support for future I2C library integration.

- **Version Control**: Added `.gitignore` to exclude build artifacts.

---

## Future Goals

- Currently doing Stub implementation, because I am lacking some hardware to fully implement everything.

- Implement `I2CReader` to communicate with the ICM-42688-P via I2C, configuring settings like �16g accelerometer range, �2000 dps gyroscope range, and 100 Hz output data rate.

- Develop a `SimulatedIMUReader` for testing without hardware.

- Design and implement a **Kalman filter** for sensor fusion.

- Add visualization of raw and fused data (e.g., using **ImPlot** or **matplotlib-cpp**).
