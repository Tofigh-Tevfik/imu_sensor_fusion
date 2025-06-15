#ifndef IMU_READER_HPP
#define IMU_READER_HPP

#include "imu_data.hpp"

class IMUReader {
    private:
        IMUData imu_data;

    public:
        IMUReader();
        virtual ~IMUReader() = 0;
        virtual bool initialize() = 0;
        virtual bool read_accelerometer(float& x, float& y, float& z) = 0;
        virtual bool read_gyroscope(float& x, float& y, float& z) = 0;
        virtual bool read_temperature(float& temp) = 0;
        virtual bool set_gyro_range(float range) = 0; // configuration method
        virtual bool set_accel_range(float range) = 0; // configuration method
};

#endif