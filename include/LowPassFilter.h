#pragma once
#include <FilterBase.h>

class LowPassFilter : public FilterBase {
    private:
        Quaternion q_;
        float filtered_gx_;
        float filtered_gy_;
        float filtered_gz_;
        float alpha_;
        float tau;
        // since the sampling time of IMU may change
        // a function to update alpha is good
        void updateAlpha(float dt);

    public:
        LowPassFilter(float cutoff_freq);
        // imu is passed by reference so less copies are made
        void update(const ImuData& imu, float dt) override;
        // this is a const function
        // the internal state of the class is not updated
        Quaternion getQuaternion() const override;
        Euler getEuler() const override;
        void reset() override;
};