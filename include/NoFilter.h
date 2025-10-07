#pragma once
#include "FilterBase.h"

class NoFilter : public FilterBase {
    private:
        Quaternion q_;

    public:
        NoFilter();

        void update(const ImuData& imu, float dt) override;
        Quaternion getQuaternion() const override;
        Euler getEuler() const override;
        void reset() override;
};