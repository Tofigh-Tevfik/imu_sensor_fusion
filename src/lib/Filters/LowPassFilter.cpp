#include <LowPassFilter.h>

void LowPassFilter::reset() {
    q_.q0 = 1;
    q_.q1 = 0;
    q_.q2 = 0;
    q_.q3 = 0;
    filtered_gx_ = 0.f;
    filtered_gy_ = 0.f;
    filtered_gz_ = 0.f;
}

Euler LowPassFilter::getEuler() const {
    Euler e;

    e.roll = atan2(2*(q_.q0*q_.q1 + q_.q2*q_.q3), 1 - 2*(q_.q1*q_.q1 + q_.q2*q_.q2));
    e.pitch = asin(-2*(q_.q1*q_.q3 - q_.q0*q_.q2));
    e.yaw = atan2(2*(q_.q0*q_.q3 + q_.q1*q_.q2), 1 - 2*(q_.q2*q_.q2 + q_.q3*q_.q3));

    return e;
}

Quaternion LowPassFilter::getQuaternion() const {
    return q_;
}

void LowPassFilter::update(const ImuData& imu, float dt) {
    updateAlpha(dt);

    filtered_gx_ = (1.0f - alpha_) * filtered_gx_ + alpha_ * imu.gx;
    filtered_gy_ = (1.0f - alpha_) * filtered_gy_ + alpha_ * imu.gy;
    filtered_gz_ = (1.0f - alpha_) * filtered_gz_ + alpha_ * imu.gz;

    float half_dt = .5f * dt;

    float qw = q_.q0;
    float qx = q_.q1;
    float qy = q_.q2;
    float qz = q_.q3;

    float gx = filtered_gx_;
    float gy = filtered_gy_;
    float gz = filtered_gz_;

    float dw = -qx*gx - qy*gy - qz*gz;
    float dx =  qw*gx + qy*gz - qz*gy;
    float dy =  qw*gy - qx*gz + qz*gx;
    float dz =  qw*gz + qx*gy - qy*gx;

    q_.q0 += half_dt * dw;
    q_.q1 += half_dt * dx;
    q_.q2 += half_dt * dy;
    q_.q3 += half_dt * dz;

    q_.normalize();
}

LowPassFilter::LowPassFilter(float cutoff_freq) {
    reset();
    tau = 1.0f / (2.0f * M_PI * cutoff_freq);
    alpha_ = 0.f;
}

void LowPassFilter::updateAlpha(float dt) {
    if (dt > 0.0f) {
        alpha_ = dt / (dt + tau);
    } else {
        alpha_ = 1.0f; // Use raw data if dt is invalid
    }
}




