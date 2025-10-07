#pragma once
#include <Types.h>
#include <FilterBase.h>

class MahonyFilter : public FilterBase {
    private:
        Quaternion q_;
        // proportional and intergral coef.
        float Kp_;
        float Ki_;
        // integrated errors
        float iBx;
        float iBy;
        float iBz;
        // anti windup clamp
        const float maxInt = 0.5f;

        void integrateGyro(const ImuData& imu, float dt);

    public:
        MahonyFilter(const float Kp, const float Ki);
        void update(const ImuData& imu, float dt) override;
        Quaternion getQuaternion() const override;
        Euler getEuler() const override;
        void reset() override;
};