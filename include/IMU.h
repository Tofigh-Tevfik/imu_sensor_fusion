#pragma once
#include <Types.h>

class IMU {
    private:
        // imu can have addresses 0x68 or 0x69
        // will be checked and verfied
        uint8_t imuAddr = 0x68;
        ImuRawData raw_data;
        ImuData scaled_data;
        uint8_t SDA_PIN;
        uint8_t SCL_PIN;
        uint8_t INT1_PIN;
        // registers
        const uint8_t REG_WHO_I_AM = 0x75;
        const uint8_t REG_DEVICE_CONFIG = 0x11;
        const uint8_t REG_PWR_MGMT0 = 0x4E;
        const uint8_t REG_TEMP_DATA1 = 0x1D;
        const uint8_t REG_GYRO_CONFIG0 = 0x4F;
        const uint8_t REG_ACCEL_CONFIG0 = 0x50;
        const uint8_t REG_INT_CONFIG   = 0x14; 
        const uint8_t REG_INT_SOURCE0  = 0x65; 
        // scaling factors
        float ACCEL_FSR_G;
        float GYRO_FSR_DPS;
        const float g = 9.81f;
        const float DIGITAL_CONVERTER = 32768.f;
        // interrupt data ready
        static volatile bool dataReady;
        // private helpers
        bool i2cWrite(uint8_t reg, uint8_t val);
        bool i2cRead(uint8_t reg, uint8_t* buf, size_t len);
        int16_t be16(uint8_t* p);
        bool WhoAmI();
        static void IRAM_ATTR isrHandler();

    public:
        IMU(uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t INT1_PIN);
        bool begin();
        void setConfig(uint8_t accel_range, uint8_t gyro_range, float accel_fsr, float gyro_fsr);
        ImuRawData readRaw();
        ImuData readScaled();
        bool available();
};