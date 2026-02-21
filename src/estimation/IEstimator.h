#pragma once

namespace atabey::estimation {

    class IEstimator {
    public:
        virtual ~IEstimator() = default;

        virtual bool init() = 0;
        virtual void update(float dt) = 0;

        virtual float getRoll() const = 0;
        virtual float getPitch() const = 0;
        virtual float getYaw() const = 0;

        virtual float getRollRate() const = 0;
        virtual float getPitchRate() const = 0;
        virtual float getYawRate() const = 0;
    };

}