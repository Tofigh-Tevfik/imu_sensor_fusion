#include <ComplementaryFilter.h>
#include <cmath>

void ComplementaryFilter::update(const ImuData& imu, float dt) {
    updateAlpha(dt);    
    
    // low pass over accel
    filtered_ax_ = (1.0f - alpha_) * filtered_ax_ + alpha_ * imu.ax;
    filtered_ay_ = (1.0f - alpha_) * filtered_ay_ + alpha_ * imu.ay;
    filtered_az_ = (1.0f - alpha_) * filtered_az_ + alpha_ * imu.az;

    // convert both accel and gyro to quaternions
    // then blend using weights
    // high pass gyro
    float half_dt = 0.5f * dt;
    float qw = q_.q0;
    float qx = q_.q1;
    float qy = q_.q2;
    float qz = q_.q3;

    float gx = imu.gx;
    float gy = imu.gy;
    float gz = imu.gz;

    float dw = -qx * gx - qy * gy - qz * gz;
    float dx = qw * gx + qy * gz - qz * gy;
    float dy = qw * gy - qx * gz + qz * gx;
    float dz = qw * gz + qx * gy - qy * gx;

    Quaternion q_gyro;
    q_gyro.q0 = qw + half_dt * dw;
    q_gyro.q1 = qx + half_dt * dx;
    q_gyro.q2 = qy + half_dt * dy;
    q_gyro.q3 = qz + half_dt * dz;
    q_gyro.normalize(); // Normalize to ensure unit quaternion

    // low pass accel
    float norm = sqrt(filtered_ax_ * filtered_ax_ + filtered_ay_ * filtered_ay_ + filtered_az_ * filtered_az_);
    Quaternion q_accel = q_;
    if (norm > 0.1f) { 
        // avoid dividing by zero, in free fall case
        float ax_norm = filtered_ax_ / norm;
        float ay_norm = filtered_ay_ / norm;
        float az_norm = filtered_az_ / norm;

        Euler e_accel;

        // quaternion from gravity (assuming z-up world frame [0 0 -1])
        e_accel.roll = atan2(ay_norm, az_norm);
        e_accel.pitch = -atan2(ax_norm, sqrt(ay_norm * ay_norm + az_norm * az_norm));
        // yaw is not observable from accel
        // so we keep the yaw the same as measured from gyro
        e_accel.yaw = quat2eul(q_gyro).yaw;
        // converting back to euler
        q_accel = eul2quat(e_accel);
    }
    // blending together
    // for blending it is better and more accurate
    // to use slerp (spherical linear interpolation)
    // however slerp is a bit more computationally heavy
    // and in small angles nlerp is very close to slerp
    // so we use nlerp here for faster computation
    Quaternion q_fused = nlerp(q_gyro, q_accel, alpha_);

    q_ = q_fused;
}

Quaternion ComplementaryFilter::getQuaternion() const {
    return q_;
}

Euler ComplementaryFilter::getEuler() const {
    Euler e;

    e.roll = atan2(2*(q_.q0*q_.q1 + q_.q2*q_.q3), 1 - 2*(q_.q1*q_.q1 + q_.q2*q_.q2));
    e.pitch = asin(-2*(q_.q1*q_.q3 - q_.q0*q_.q2));
    e.yaw = atan2(2*(q_.q0*q_.q3 + q_.q1*q_.q2), 1 - 2*(q_.q2*q_.q2 + q_.q3*q_.q3));

    return e;
}

ComplementaryFilter::ComplementaryFilter(const float lowpass_cutoff_freq) {
    reset();
    tau = 1.0f / (2.0f * M_PI * lowpass_cutoff_freq);
}

void ComplementaryFilter::reset() {
    q_.q0 = 1;
    q_.q1 = 0;
    q_.q2 = 0;
    q_.q3 = 0;
    // accel data
    filtered_ax_ = 0.f;
    filtered_ay_ = 0.f;
    filtered_az_ = 9.81f;
}

void ComplementaryFilter::updateAlpha(float dt) {
    if (dt > 0.0f) {
        alpha_ = dt / (dt + tau);
    } else {
        // use raw data if dt is invalid
        alpha_ = 1.0f;
    }
}

Euler ComplementaryFilter::quat2eul(const Quaternion q) const {
    Euler e;

    e.roll = atan2(2*(q.q0*q.q1 + q.q2*q.q3), 1 - 2*(q.q1*q.q1 + q.q2*q.q2));
    e.pitch = asin(-2*(q.q1*q.q3 - q.q0*q.q2));
    e.yaw = atan2(2*(q.q0*q.q3 + q.q1*q.q2), 1 - 2*(q.q2*q.q2 + q.q3*q.q3));

    return e;
};

Quaternion ComplementaryFilter::eul2quat(const Euler e) const {
    Quaternion q;

    float cy = cos(e.yaw * 0.5f);
    float sy = sin(e.yaw * 0.5f);
    float cp = cos(e.pitch * 0.5f);
    float sp = sin(e.pitch * 0.5f);
    float cr = cos(e.roll * 0.5f);
    float sr = sin(e.roll * 0.5f);

    q.q0 = cr * cp * cy + sr * sp * sy; // w
    q.q1 = sr * cp * cy - cr * sp * sy; // x
    q.q2 = cr * sp * cy + sr * cp * sy; // y
    q.q3 = cr * cp * sy - sr * sp * cy; // z

    q.normalize();
    return q;
}

Quaternion ComplementaryFilter::nlerp(const Quaternion& q1, const Quaternion& q2, float t) {
    Quaternion q2_copy = q2;
    // if dot product of q1 and q2 is negative
    // we have flip q2 to ensure shortest path
    if (q1.dot(q2) < 0.0f) {
        q2_copy = -q2_copy;
    }

    // linear interpolation
    Quaternion result = (q1 * (1.f - t))  + (q2_copy * t);

    result.normalize();
    return result;
}