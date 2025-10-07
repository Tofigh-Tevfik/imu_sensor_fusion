#pragma once
#include <Types.h>

class FilterBase {
    public:
        virtual void update(const ImuData& imu, float dt) = 0;
        virtual Quaternion getQuaternion() const = 0;
        virtual Euler getEuler() const = 0;
        virtual void reset() = 0;
        virtual ~FilterBase() {}
};