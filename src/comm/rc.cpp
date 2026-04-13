#include "rc.h"
#include <Arduino.h>

#include "../utils/MathUtils.h"

using namespace atabey::utils;

namespace atabey {
    namespace comm {
        volatile int Receiver::rollValue = 0;
        volatile int Receiver::pitchValue = 0;
        volatile int Receiver::throttleValue = 0;
        volatile int Receiver::yawValue = 0;

        volatile unsigned long Receiver::rollStart = 0;
        volatile unsigned long Receiver::pitchStart = 0;
        volatile unsigned long Receiver::throttleStart = 0;
        volatile unsigned long Receiver::yawStart = 0;
        
        Receiver::Receiver() {}

        void Receiver::init() {
        attachInterrupt(digitalPinToInterrupt(ROLL_PIN), rollISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PITCH_PIN), pitchISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(YAW_PIN), yawISR, CHANGE);
        }

        int16_t Receiver::getRoll() {
            return map(rollValue, 945, 1975, -100, 100);
        }

        int16_t Receiver::getPitch() {
            return map(pitchValue, 945, 1950, -100, 100);
        }

        int16_t Receiver::getYaw() { 
            return map(yawValue, 970, 1965, -100, 100); 
        }

        int16_t Receiver::getThrottle() {
            return map(throttleValue, 950, 1990, 0, 100);
        }

        int16_t Receiver::getRawRoll() {
        return rollValue;
        }

        int16_t Receiver::getRawPitch() {
        return pitchValue;
        }

        int16_t Receiver::getRawThrottle() {
        return throttleValue;
        }

        int16_t Receiver::getRawYaw() {
        return yawValue;
        }

        void Receiver::rollISR() {
        if (digitalRead(ROLL_PIN) == HIGH) {
            rollStart = micros();
        } else {
            rollValue = micros() - rollStart;
        }
        }

        void Receiver::pitchISR() {
        if (digitalRead(PITCH_PIN) == HIGH) {
            pitchStart = micros();
        } else {
            pitchValue = micros() - pitchStart;
        }
        }

        void Receiver::throttleISR() {
        if (digitalRead(THROTTLE_PIN) == HIGH) {
            throttleStart = micros();
        } else {
            throttleValue = micros() - throttleStart;
        }
        }

        void Receiver::yawISR() {
        if (digitalRead(YAW_PIN) == HIGH) {
            yawStart = micros();
        } else {
            yawValue = micros() - yawStart;
        }
        }
    }
}
