#pragma once

#include "Arduino.h"
#include "IActuator.h"

#define SERVO_MIN -20 // Derece cinsinden minimum servo açısı
#define SERVO_MAX 20  // Derece cinsinden maksimum servo açısı
#define PWM_MIN 120 // 490 Hz sinyali pwm pulse olarak kullanmak için yaptık.
#define PWM_MAX 250

namespace atabey {
    namespace drivers {

        template<uint8_t ELEVON_SOL_PIN, uint8_t ELEVON_SAG_PIN>
        class ServoPWM : public atabey::drivers::IActuator {
            public:
                ServoPWM() {}

                bool init() override {
                    pinMode(ELEVON_SOL_PIN, OUTPUT);
                    pinMode(ELEVON_SAG_PIN, OUTPUT);
                    disarm();
                    return true;
                }

                void setPosition(float _solAngle, float _sagAngle) {

                    // Gelen açıları sınırla ve PWM_MIN - PWM_MAX aralığına dönüştür
                    uint8_t solAci = (constrain(_solAngle, SERVO_MIN, SERVO_MAX) - SERVO_MIN) * (PWM_MAX - PWM_MIN) / (SERVO_MAX - SERVO_MIN) + PWM_MIN;
                    uint8_t sagAci = (constrain(_sagAngle, SERVO_MIN, SERVO_MAX) - SERVO_MIN) * (PWM_MAX - PWM_MIN) / (SERVO_MAX - SERVO_MIN) + PWM_MIN;

                    analogWrite(ELEVON_SOL_PIN, solAci);
                    analogWrite(ELEVON_SAG_PIN, sagAci);
                }

                void disarm() {
                    analogWrite(ELEVON_SOL_PIN, 0);
                    analogWrite(ELEVON_SAG_PIN, 0);
                }

            };

    }
}