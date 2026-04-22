#pragma once

#include <stdint.h>

#define ROLL_PIN 2
#define PITCH_PIN 3
#define THROTTLE_PIN 18
#define YAW_PIN 19

#define MIN_PWM 992
#define MAX_PWM 1984

namespace atabey {
    namespace comm {
        class Receiver {
            private:
                static volatile int rollValue;
                static volatile int pitchValue;
                static volatile int throttleValue;
                static volatile int yawValue;

                static volatile unsigned long rollStart;
                static volatile unsigned long pitchStart;
                static volatile unsigned long throttleStart;
                static volatile unsigned long yawStart;

                static void rollISR();
                static void pitchISR();
                static void throttleISR();
                static void yawISR();

            public:
                Receiver();
                void init();
                
                int16_t getRoll();
                int16_t getPitch();
                int16_t getThrottle();
                int16_t getYaw();
                
                int16_t getRawRoll();
                int16_t getRawPitch();
                int16_t getRawThrottle();
                int16_t getRawYaw();
        };
    }
}

