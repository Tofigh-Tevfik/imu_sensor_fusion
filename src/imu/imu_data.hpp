#ifndef IMU_DATA_HPP
#define IMU_DATA_HPP

#include <cstdint>

// struct of accelerameter data
struct accel {
    int32_t x;
    int32_t y;
    int32_t z;
};

// struct for gyroscope data
struct gyro {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct IMUData {
    uint64_t timestamp;
    int32_t temperature;
    gyro gyro_data;
    accel accel_data;
};

#endif