#pragma once

namespace atabey::drivers {

    class IActuator {
    public:
        virtual ~IActuator() = default;

        virtual bool init() = 0;

        virtual void setAileron(float value) = 0;
        virtual void setElevator(float value) = 0;
        virtual void setRudder(float value) = 0;
        virtual void setThrottle(float value) = 0;

        virtual void disarm() = 0;
    };

}