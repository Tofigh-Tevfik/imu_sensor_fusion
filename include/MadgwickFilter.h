#pragma once
#include <Types.h>
#include <FilterBase.h>

class MadgwickFilter : public FilterBase {
    private:
        Quaternion q_;
        // step size alpha_
        // called Learning Rate by ML community
        float alpha_;
        void skipUpdate(float dw, float dx, float dy, float dz, float dt);

    public:
        MadgwickFilter(const float stepSize);
        void update(const ImuData& imu, float dt) override;
        Quaternion getQuaternion() const override;
        Euler getEuler() const override;
        void reset() override;
};