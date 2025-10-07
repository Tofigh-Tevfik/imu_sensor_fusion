#pragma once
#include <Types.h>
#include <FilterBase.h>
#include <BasicLinearAlgebra.h>

class ExtendedKF : public FilterBase {
    public:
        ExtendedKF(BLA::Matrix<6,6> Q, BLA::Matrix<3,3> R, BLA::Matrix<7, 7> P);
        void update(const ImuData& imu, float dt) override;
        Quaternion getQuaternion() const override;
        Euler getEuler() const override;
        void reset() override;

    private:
        // estimated state
        // [qw, qx, qy, qz, bx, by, bz]^T [7, 1]
        BLA::Matrix<7,1> x;
        // kalman gain
        BLA::Matrix<7,3> K;
        // Prcoess noise variance
        BLA::Matrix<6,6> Q;
        // measurement noise variance
        BLA::Matrix<3,3> R;
        // Process covariance P
        BLA::Matrix<7,7> P;
        // Estimated orientation
        BLA::Matrix<4,1> q;
        // Estimated gyro bias
        BLA::Matrix<3,1> b;
        void timeUpdate(const ImuData& imu, float dt);
        void measurementUpdate(const ImuData& imu, float dt);
};