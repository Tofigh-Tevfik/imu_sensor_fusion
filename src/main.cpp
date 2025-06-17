#include "imu/i2c_reader.cpp"
#include <iostream>

int main() {
    std::cout << "IMU Sensor fusion" << std::endl;

    try {
        I2CReader reader("/dev/i2c-1", 0x68);
        if (reader.initialize()) {
            std::cout << "I2C reader initialized" << std::endl;
        } else {
            std::cout << "I2CReader initializiation failed" << std::endl;
        }
    } catch (const std::exception& e){
        std::cerr << "error: " << e.what() << std::endl;
    }

    return 0;
}