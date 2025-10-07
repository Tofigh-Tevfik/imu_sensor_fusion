#include <MahonyFilter.h>

void MahonyFilter::update(const ImuData& imu, float dt) {
    if (dt <= 0.0f) {
        return;
    }
    // imu data
    float gx = imu.gx;
    float gy = imu.gy;
    float gz = imu.gz;
    // accel terms
    float ax = imu.ax;
    float ay = imu.ay;
    float az = imu.az;
    // normalizing accel terms
    // we will get the unit vector pointing at gravity 
    float a_norm = sqrt(ax*ax + ay*ay + az*az);
    if (a_norm < 1e-6f) {
        integrateGyro(imu, dt);
        return;
    }
    ax /= a_norm; ay /= a_norm; az /= a_norm;

    float qw = q_.q0;
    float qx = q_.q1;
    float qy = q_.q2;
    float qz = q_.q3;

    // vector pointing at gravity measured from gyro
    float vx = 2.0f * (qx*qz - qw*qy);
    float vy = 2.0f * (qw*qx + qy*qz);
    float vz = qw*qw - qx*qx - qy*qy + qz*qz;
    // error = a x v
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);
    // integration and anti windup
    iBx += Ki_ * ex * dt;
    iBy += Ki_ * ey * dt;
    iBz += Ki_ * ez * dt;
    float int_mag = sqrt(iBx*iBx + iBy*iBy + iBz*iBz);
    if (int_mag > maxInt) {
        iBx *= maxInt/int_mag;
        iBy *= maxInt/int_mag;
        iBz *= maxInt/int_mag;
    }

    gx += iBx + Kp_ * ex;
    gy += iBy + Kp_ * ey;
    gz += iBz + Kp_ * ez;

    float dw = 0.5f*(-qx * gx - qy * gy - qz * gz);
    float dx = 0.5f*(qw * gx + qy * gz - qz * gy);
    float dy = 0.5f*(qw * gy - qx * gz + qz * gx);
    float dz = 0.5f*(qw * gz + qx * gy - qy * gx);

    q_.q0 += dw * dt;
    q_.q1 += dx * dt;
    q_.q2 += dy * dt;
    q_.q3 += dz * dt;
    q_.normalize();
}

MahonyFilter::MahonyFilter(float Kp, float Ki) {
    reset();
    Kp_ = Kp;
    // if integral term is negative we disable integration
    Ki_ = (Ki < 0.0f) ? 0.0f : Ki;
}

Quaternion MahonyFilter::getQuaternion() const {
    return q_;
}

Euler MahonyFilter::getEuler() const {
    Euler e;

    e.roll = atan2(2*(q_.q0*q_.q1 + q_.q2*q_.q3), 1 - 2*(q_.q1*q_.q1 + q_.q2*q_.q2));
    e.pitch = asin(-2*(q_.q1*q_.q3 - q_.q0*q_.q2));
    e.yaw = atan2(2*(q_.q0*q_.q3 + q_.q1*q_.q2), 1 - 2*(q_.q2*q_.q2 + q_.q3*q_.q3));

    return e;
}

void MahonyFilter::integrateGyro(const ImuData& imu, float dt) {
    float qw = q_.q0;
    float qx = q_.q1;
    float qy = q_.q2;
    float qz = q_.q3;

    float gx = imu.gx;
    float gy = imu.gy;
    float gz = imu.gz;

    // quaternion derivatives
    float dw = 0.5f*(-qx * gx - qy * gy - qz * gz);
    float dx = 0.5f*(qw * gx + qy * gz - qz * gy);
    float dy = 0.5f*(qw * gy - qx * gz + qz * gx);
    float dz = 0.5f*(qw * gz + qx * gy - qy * gx);

    q_.q0 += dw * dt;
    q_.q1 += dx * dt;
    q_.q2 += dy * dt;
    q_.q3 += dz * dt;
    q_.normalize();

}

void MahonyFilter::reset() {
    q_.q0 = 1.0f;
    q_.q1 = 0.0f;
    q_.q2 = 0.0f;
    q_.q3 = 0.0f;
    // set integral terms also to 0
    iBx = 0.0f;
    iBy = 0.0f;
    iBz = 0.0f;
}