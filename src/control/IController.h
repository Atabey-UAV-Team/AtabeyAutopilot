#pragma once

namespace atabey::control {

    class IController {
    public:
        virtual ~IController() = default;

        virtual bool init() = 0;
        virtual void update(float dt) = 0;

        virtual void setTarget(float roll, float pitch, float yaw) = 0;

        virtual float getAileron() const = 0;
        virtual float getElevator() const = 0;
        virtual float getRudder() const = 0;
        virtual float getThrottle() const = 0;
    };

}