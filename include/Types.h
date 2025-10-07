#pragma once
#include <Arduino.h>


struct ImuRawData {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    int16_t tempC_raw;
};

struct ImuData {
    float ax, ay, az; // m/s^2
    float gx, gy, gz; // rad/s
    float tempC;      // Celsius
};

struct Quaternion {
    float q0, q1, q2, q3;

    void normalize() {
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm > 0) {
            q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
        }
    }

    // Dot product
    float dot(const Quaternion& q) const {
        return q0*q.q0 + q1*q.q1 + q2*q.q2 + q3*q.q3;
    }

    // adding some operators
    // scaler multiplication
    Quaternion operator*(float s) const {
        return {q0*s, q1*s, q2*s, q3*s};
    }

    // addition
    Quaternion operator+ (const Quaternion& q) const {
        return {q0 + q.q0, q1 + q.q1, q2 + q.q2, q3 + q.q3};
    }

    // Subtraction
    Quaternion operator-() const {
        return {-q0, -q1, -q2, -q3};
    }

    // Quaternion multiplication (Hamilton product)
    Quaternion multiply(const Quaternion& r) const {
        return {
            q0*r.q0 - q1*r.q1 - q2*r.q2 - q3*r.q3, 
            q0*r.q1 + q1*r.q0 + q2*r.q3 - q3*r.q2,  
            q0*r.q2 - q1*r.q3 + q2*r.q0 + q3*r.q1,  
            q0*r.q3 + q1*r.q2 - q2*r.q1 + q3*r.q0   
        };
    }

    // Operator overload for quaternion multiplication
    Quaternion operator*(const Quaternion& r) const {
        return multiply(r);
    }
};

struct Euler {
    float roll, pitch, yaw;
};