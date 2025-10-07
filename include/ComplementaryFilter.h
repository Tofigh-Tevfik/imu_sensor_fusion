#pragma once
#include <Types.h>
#include <FilterBase.h>

class ComplementaryFilter : public FilterBase {
    private:
        Quaternion q_;
        // filtered accel data
        float filtered_ax_;
        float filtered_ay_;
        float filtered_az_;
        // low pass parameter
        float alpha_;
        float tau;

        void updateAlpha(float dt);
        Euler quat2eul(const Quaternion q) const;
        Quaternion eul2quat(const Euler e) const;
        // normalize linear interpolation
        Quaternion nlerp(const Quaternion& q1, const Quaternion& q2, float t);

    public:
        ComplementaryFilter(const float lowpass_cutoff_freq);
        void update(const ImuData& imu, float dt) override;
        Quaternion getQuaternion() const override;
        Euler getEuler() const override;
        void reset() override;
};