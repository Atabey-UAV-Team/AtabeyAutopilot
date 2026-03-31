#pragma once

#include "ServoDriver.h"

// Servo fiziksel aralığı
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_TRIM_US 1500

// Elevon çalışma aralığı
#define ELEVON_MIN_ANGLE -20.0f
#define ELEVON_MAX_ANGLE 20.0f

namespace atabey {
    namespace drivers {

        template<class Driver>
        class ServoPWM {
            private:
                Driver& _driver;
                uint8_t _ch1, _ch2;

                uint16_t angleToUs(float angle);

            public:
                ServoPWM(Driver& driver, uint8_t ch1, uint8_t ch2);

                void init();
                void setPosition(float solAngle, float sagAngle);
                void disarm();
        };

    }
}

#include "servo.tpp"