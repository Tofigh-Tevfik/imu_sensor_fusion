#include <Wire.h>
#include <IMU.h>
#include <math.h>

// constructor
IMU::IMU (uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t INT1_PIN) {
    this->SDA_PIN = SDA_PIN;
    this->SCL_PIN = SCL_PIN;
    this->INT1_PIN = INT1_PIN;
    Wire.begin(SDA_PIN, SCL_PIN, 500000);
}

bool IMU::begin() {
    if (WhoAmI()) {
        pinMode(INT1_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(INT1_PIN), IMU::isrHandler, RISING);
        return true;
    }
    return false;
}

void IMU::setConfig(uint8_t accel_range, uint8_t gyro_range, float accel_fsr, float gyro_fsr) {
    ACCEL_FSR_G = accel_fsr;
    GYRO_FSR_DPS = gyro_fsr;
    i2cWrite(REG_DEVICE_CONFIG, 0x01); // soft reset of all registers
    delay(2);
    // setting gyro and accel in low noise mode
    // 00001111 bits 1:0 -> accel in low noise mode
    // 00001111 bits 3:2 -> gyro in low noise mode
    i2cWrite(REG_PWR_MGMT0, 0x0F);
    delay(10);
    i2cWrite(REG_GYRO_CONFIG0, gyro_range);
    i2cWrite(REG_ACCEL_CONFIG0, accel_range);
    delay(10);
    i2cWrite(REG_INT_CONFIG , 0x03);
    i2cWrite(REG_INT_SOURCE0, 0x08);
    delay(10);
}

ImuRawData IMU::readRaw() {
    uint8_t buf[14];
    if (i2cRead(REG_TEMP_DATA1, buf, sizeof(buf))) {
        raw_data.tempC_raw = be16(&buf[0]);
        raw_data.ax_raw = be16(&buf[2]);
        raw_data.ay_raw = be16(&buf[4]);
        raw_data.az_raw = be16(&buf[6]);
        raw_data.gx_raw = be16(&buf[8]);
        raw_data.gy_raw = be16(&buf[10]);
        raw_data.gz_raw = be16(&buf[12]);
        
    } else {
        Serial.println("Could not read IMU\n");
    }

    return raw_data;
}

ImuData IMU::readScaled() {
    readRaw();

    // temperature
    scaled_data.tempC = ( (float) raw_data.tempC_raw / 132.48f ) + 25.0f;
    // accelerometer
    scaled_data.ax = ( (float) raw_data.ax_raw * (ACCEL_FSR_G / DIGITAL_CONVERTER)) * g;
    scaled_data.ay = ( (float) raw_data.ay_raw * (ACCEL_FSR_G / DIGITAL_CONVERTER)) * g;
    scaled_data.az = ( (float) raw_data.az_raw * (ACCEL_FSR_G / DIGITAL_CONVERTER)) * g;
    // gyro
    scaled_data.gx = ( (float) raw_data.gx_raw * (GYRO_FSR_DPS / DIGITAL_CONVERTER)) * (M_PI / 180.0f);;
    scaled_data.gy = ( (float) raw_data.gy_raw * (GYRO_FSR_DPS / DIGITAL_CONVERTER)) * (M_PI / 180.0f);;
    scaled_data.gz = ( (float) raw_data.gz_raw * (GYRO_FSR_DPS / DIGITAL_CONVERTER)) * (M_PI / 180.0f);;

    return scaled_data;
}

bool IMU::available() {
    if (dataReady) {
        delayMicroseconds(100);
        dataReady = false;
        return true;
    }
    return false;
}

// private helpers
bool IMU::WhoAmI() {
    for (uint8_t addr : {0x68, 0x69}) {
        imuAddr = addr;
        uint8_t who = 0;
        if (i2cRead(REG_WHO_I_AM, &who, 1) && who == 0x47) {
            return true;
        }
    }
    return false;
}

bool IMU::i2cWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool IMU::i2cRead(uint8_t reg, uint8_t* buf, size_t len) {
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    int status = Wire.endTransmission(false);
    if (status != 0) {
        Serial.printf("i2cRead endTransmission failed: %d\n", status);
        return false;
    }
    // retrying 3 times if reading fails
    for (int attempt = 0; attempt < 3; attempt++) {
        size_t n = Wire.requestFrom(imuAddr, len);
        if (n == len) {
            for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
            return true;
        }
        Serial.printf("i2cRead requestFrom failed: expected %u, got %u, attempt %d\n", len, n, attempt);
        delayMicroseconds(100); // Wait before retry
    }
    return false;
}
volatile bool IMU::dataReady = false;

void IRAM_ATTR IMU::isrHandler() {
    dataReady = true;
}

// concatenating to 8 bits
int16_t IMU::be16(uint8_t* p) { return (int16_t)((p[0]) << 8 | p[1]); }