#include <MadgwickFilter.h>

void MadgwickFilter::update(const ImuData& imu, float dt) {
    if (dt <= 0.0f) {
        return;
    }
    float qw = q_.q0;
    float qx = q_.q1;
    float qy = q_.q2;
    float qz = q_.q3;
    // imu data
    float gx = imu.gx;
    float gy = imu.gy;
    float gz = imu.gz;
    // quaternion derivatives
    float dw = 0.5f*(-qx * gx - qy * gy - qz * gz);
    float dx = 0.5f*(qw * gx + qy * gz - qz * gy);
    float dy = 0.5f*(qw * gy - qx * gz + qz * gx);
    float dz = 0.5f*(qw * gz + qx * gy - qy * gx);
    // normalizing accel data to get gravity vector direction
    float ax = imu.ax;
    float ay = imu.ay;
    float az = imu.az;
    float a_norm = sqrt(ax * ax + ay * ay + az * az);
    // if accel norm is 0 we only intergrate gyro and retrun
    if (a_norm < 1e-6f) {
        skipUpdate(dw, dx, dy, dz, dt);
        return;
    }
    // else we will continue
    ax /= a_norm; ay /= a_norm; az /= a_norm;
    // objective function
    float f1 = 2.0f * (qx * qz - qw * qy) - ax;
    float f2 = 2.0f * (qw * qx + qy * qz) - ay;
    float f3 = 2.0f * (0.5f - qx*qx - qy*qy) - az;
    // gradient descent direction
    float s0 = -2.0f * qy * f1 + 2.0f * qx * f2;
    float s1 = 2.0f * qw * f2 + 2.0f * qz * f1 - 2.0f * qx * f3;
    float s2 = -2.0f * qw * f1 + 2.0f * qz * f2 - 2.0f * qy * f3;
    float s3 = 2.0f * qx * f1 + 2.0f * qy * f2;
    // normalizing s
    float s_norm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (s_norm < 1e-6f) {
        skipUpdate(dw, dx, dy, dz, dt);
        return;
    }
    s0 /= s_norm; s1 /= s_norm; s2 /= s_norm; s3 /= s_norm;
    // steping in the descent direction
    // with the step size alpha_
    dw -= alpha_ * s0;
    dx -= alpha_ * s1;
    dy -= alpha_ * s2;
    dz -= alpha_ * s3;

    q_.q0 += dw * dt;
    q_.q1 += dx * dt;
    q_.q2 += dy * dt;
    q_.q3 += dz * dt;
    q_.normalize();
}

MadgwickFilter::MadgwickFilter(const float stepSize) {
    alpha_ = stepSize;
    reset();
}

Quaternion MadgwickFilter::getQuaternion() const {
    return q_;
}

Euler MadgwickFilter::getEuler() const {
    Euler e;

    e.roll = atan2(2*(q_.q0*q_.q1 + q_.q2*q_.q3), 1 - 2*(q_.q1*q_.q1 + q_.q2*q_.q2));
    e.pitch = asin(-2*(q_.q1*q_.q3 - q_.q0*q_.q2));
    e.yaw = atan2(2*(q_.q0*q_.q3 + q_.q1*q_.q2), 1 - 2*(q_.q2*q_.q2 + q_.q3*q_.q3));

    return e;
}

void MadgwickFilter::skipUpdate(float dw, float dx, float dy, float dz, float dt) {
    q_.q0 += dw * dt;
    q_.q1 += dx * dt;
    q_.q2 += dy * dt;
    q_.q3 += dz * dt;
    q_.normalize();
}

void MadgwickFilter::reset() {
    q_.q0 = 1.0f;
    q_.q1 = 0.0f;
    q_.q2 = 0.0f;
    q_.q3 = 0.0f;
}