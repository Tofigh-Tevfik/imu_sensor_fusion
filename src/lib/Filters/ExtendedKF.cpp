#include <ExtendedKF.h>

void ExtendedKF::update(const ImuData& imu, float dt) {
    // predict
    timeUpdate(imu, dt);
    // update
    measurementUpdate(imu, dt);
}

void ExtendedKF::measurementUpdate(const ImuData& imu, float dt) {
    float qw = x(0,0); float qx = x(1,0); float qy = x(2,0); float qz = x(3,0);
    // rotation matrix
    BLA::Matrix<3,3> R = {1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw,
                          2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw,
                          2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy};
    // gravity vector
    float g = 9.81f;
    BLA::Matrix<3, 1> g_vec = {0, 0, g};
    // measurment vector
    BLA::Matrix<3, 1> z = {imu.ax, imu.ay, imu.az};
    // measurement residual
    BLA::Matrix<3, 1> y = z - ~R*g_vec;
    // jacobian of the update dynamics
    BLA::Matrix<3, 7> H = {-2*g*qy, 2*g*qz, -2*g*qw, 2*g*qx, 0, 0, 0,
                           2*g*qx, 2*g*qw, 2*g*qz, 2*g*qy, 0, 0, 0,
                           0, -4*g*qx, -4*g*qy, 0, 0, 0, 0};
    // innovation covariance
    BLA::Matrix<3, 3> Sk = H * P * ~H + R;
    // near optimal kalman gain
    K = P * ~H * BLA::Inverse(Sk);
    x = x + K * y;
    // normalization
    float q_norm = sqrt(x(0,0)*x(0,0) + x(1,0)*x(1,0) + x(2,0)*x(2,0) + x(3,0)*x(3,0));
    if (q_norm > 1e-6f) {
        x(0, 0) /= q_norm;
        x(1, 0) /= q_norm;
        x(2, 0) /= q_norm;
        x(3, 0) /= q_norm;
    }
    q(0, 0) = x(0, 0); q(1, 0) = x(1, 0); q(2, 0) = x(2, 0); q(3, 0) = x(3, 0);
    P = (BLA::Eye<7, 7>() - K * H) * P;
}

void ExtendedKF::timeUpdate(const ImuData& imu, float dt) {
    BLA::Matrix<3,1> w = {imu.gx, imu.gy, imu.gz};
    w = w - b; 
    // q: process update dynamics
    BLA::Matrix<4,4> omega = {0, -w(0,0), -w(1,0), -w(2,0),
                               w(0,0), 0, w(2,0), -w(1,0),
                               w(1,0), -w(2,0), 0, w(0,0),
                               w(2,0), w(1,0), -w(0,0), 0};
    BLA::Matrix<4,1> qdot = 0.5f * omega * q;
    BLA::Matrix<3,1> bdot = BLA::Zeros<3,1>();
    // process dynamic equations
    BLA::Matrix<7,1> f;
    f(0,0) = qdot(0,0); f(1,0) = qdot(1,0); f(2,0) = qdot(2,0); f(3,0) = qdot(3,0);
    f(4,0) = bdot(0,0); f(5,0) = bdot(1,0); f(6,0) = bdot(2,0);

    x = x + f * dt;
    // normalization
    float q_norm = sqrt(x(0,0)*x(0,0) + x(1,0)*x(1,0) + x(2,0)*x(2,0) + x(3,0)*x(3,0));
    if (q_norm > 1e-6f) {
        x(0, 0) /= q_norm;
        x(1, 0) /= q_norm;
        x(2, 0) /= q_norm;
        x(3, 0) /= q_norm;
    }
    q(0, 0) = x(0, 0); q(1, 0) = x(1, 0); q(2, 0) = x(2, 0); q(3, 0) = x(3, 0);
    float qw = x(0,0); float qx = x(1,0); float qy = x(2,0); float qz = x(3,0);
    // gyro data
    float gx = imu.gx; float gy = imu.gy; float gz = imu.gz;
    // gyro bias
    float bx = x(4, 0); float by = x(5, 0); float bz = x(6, 0);
    BLA::Matrix<7,7> F = {1.0f, 0.5f*dt*(bx-gx), 0.5f*dt*(by-gy), 0.5f*dt*(bz-gz), 0.5f*dt*(qx),  0.5f*(dt*qy),  0.5f*(dt*qz),
                          -(0.5f*dt*(bx-gx)), 1.0f, -0.5f*(dt*(bz-gz)),  0.5f*dt*(by-gy), -0.5f*(dt*qw),  0.5f*(dt*qz), -0.5f*(dt*qy),
                          -0.5f*(dt*(by-gy)), 0.5f*dt*(bz-gz), 1.0f, -0.5f*(dt*(bx-gx)), -0.5f*(dt*qz), -0.5f*(dt*qw), 0.5f*(dt*qx),
                          -0.5f*(dt*(bz-gz)), -0.5f*(dt*(by-gy)),  0.5f*dt*(bx-gx), 1, 0.5f*(dt*qy), -0.5f*(dt*qx), -0.5f*(dt*qw),
                          0, 0, 0, 0, 1.0f, 0, 0,
                          0, 0, 0, 0, 0, 1.0f, 0,
                          0, 0, 0, 0, 0, 0, 1.0f};
    
    BLA::Matrix<7,6> G = {
        0.5f*dt, 0,       0,       0, 0, 0,
        0,       0.5f*dt, 0,       0, 0, 0,
        0,       0,       0.5f*dt, 0, 0, 0,
        0,       0,       0,       0, 0, 0,
        0,0,0, 1,0,0,
        0,0,0, 0,1,0,
        0,0,0, 0,0,1
    };  

    P = F * P * ~F + G * Q * ~G;
} 

ExtendedKF::ExtendedKF(BLA::Matrix<6,6> Q, BLA::Matrix<3,3> R, BLA::Matrix<7, 7> P) {
    reset();
    this->Q = Q;
    this->R = R;
    this->P = P; 
}

void ExtendedKF::reset() {
    q = BLA::Zeros<4,1>();
    q(0, 0) = 1.0f;
    K = BLA::Zeros<7,3>();
    P = BLA::Zeros<7,7>();
    x = BLA::Zeros<7,1>();
    x(0, 0) = q(0, 0);
    b = BLA::Zeros<3,1>();
}

Quaternion ExtendedKF::getQuaternion() const {
    Quaternion q_;
    q_.q0 = x(0,0);
    q_.q1 = x(1,0);
    q_.q2 = x(2,0);
    q_.q3 = x(3,0);

    return q_;
}

Euler ExtendedKF::getEuler() const {
    Quaternion q_ = getQuaternion();

    Euler e;

    e.roll = atan2(2*(q_.q0*q_.q1 + q_.q2*q_.q3), 1 - 2*(q_.q1*q_.q1 + q_.q2*q_.q2));
    e.pitch = asin(-2*(q_.q1*q_.q3 - q_.q0*q_.q2));
    e.yaw = atan2(2*(q_.q0*q_.q3 + q_.q1*q_.q2), 1 - 2*(q_.q2*q_.q2 + q_.q3*q_.q3));

    return e;
}