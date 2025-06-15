#ifndef I2C_READER_HPP
#define I2C_READER_HPP

#include "imu_reader.hpp"
#include <string>

class I2CReader : public IMUReader
{
    private:
        int i2c_fd;
        uint8_t i2c_address;

    public:
        I2CReader(const std::string& i2c_bus, uint8_t address);
        virtual ~I2CReader() override;
        virtual bool initialize() override;
        virtual bool read_accelerometer(float& x, float& y, float& z) override;
        virtual bool read_gyroscope(float& x, float& y, float& z) override;
        virtual bool read_temperature(float& temp) override;
        virtual bool set_gyro_range(float range) override;
        virtual bool set_accel_range(float range) override;
        bool write_register(uint8_t reg, uint8_t value); // helper method
        bool read_register(uint8_t reg, uint8_t* data, size_t len); // another helper method
};

#endif