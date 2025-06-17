#include "i2c_reader.hpp"
#include <stdexcept>

// stub impelementation
I2CReader::I2CReader(const std::string& i2c_bus, uint8_t address) 
    : i2c_fd(-1), i2c_address(address) {
    throw::std::runtime_error("I2CReader class not implemented");
}

I2CReader::~I2CReader() {}

bool I2CReader::initialize() {
    return false;
}

bool I2CReader::read_accelerometer(float& x, float& y, float& z) {
    x = y = z = 0.f;
    return false;
}

bool I2CReader::read_gyroscope(float& x, float& y, float& z) {
    x = y = z = 0.f;
    return false;
}

bool I2CReader::read_temperature(float& temp) {
    temp = 0.f;
    return false;
}

bool I2CReader::set_gyro_range(float range) {
    return false;
}

bool I2CReader::set_accel_range(float range) {
    return false;
}

bool I2CReader::write_register(uint8_t reg, uint8_t value) {
    return false;
}

bool I2CReader::read_register(uint8_t reg, uint8_t* data, size_t len) {
    return false;
}