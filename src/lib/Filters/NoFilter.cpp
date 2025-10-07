#include <NoFilter.h>

NoFilter::NoFilter() {
    reset();
}

void NoFilter::update(const ImuData& imu, float dt) {
    float half_dt = .5f * dt;

    float qw = q_.q0;
    float qx = q_.q1;
    float qy = q_.q2;
    float qz = q_.q3;

    float gx = imu.gx;
    float gy = imu.gy;
    float gz = imu.gz;

    float dw = -qx*gx - qy*gy - qz*gz;
    float dx =  qw*gx + qy*gz - qz*gy;
    float dy =  qw*gy - qx*gz + qz*gx;
    float dz =  qw*gz + qx*gy - qy*gx;

    q_.q0 += half_dt * dw;
    q_.q1 += half_dt * dx;
    q_.q2 += half_dt * dy;
    q_.q3 += half_dt * dz;

    q_.normalize();
};

Quaternion NoFilter::getQuaternion() const {
    return q_;
};

Euler NoFilter::getEuler() const {
    Euler e;

    e.roll = atan2(2*(q_.q0*q_.q1 + q_.q2*q_.q3), 1 - 2*(q_.q1*q_.q1 + q_.q2*q_.q2));
    e.pitch = asin(-2*(q_.q1*q_.q3 - q_.q0*q_.q2));
    e.yaw = atan2(2*(q_.q0*q_.q3 + q_.q1*q_.q2), 1 - 2*(q_.q2*q_.q2 + q_.q3*q_.q3));

    return e;
}

void NoFilter::reset() {
    q_.q0 = 1;
    q_.q1 = 0;
    q_.q2 = 0;
    q_.q3 = 0;
}