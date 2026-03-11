#pragma once

#include "Arduino.h"
#include "IActuator.h"

namespace atabey {
    namespace drivers {

        template<uint8_t ELEVON_SOL_PIN, uint8_t ELEVON_SAG_PIN>
        class ServoPWM : public atabey::drivers::IActuator {
            private:
                static constexpr int8_t SERVO_MIN = -20; // Derece cinsinden minimum servo açısı
                static constexpr int8_t SERVO_MAX = 20;  // Derece cinsinden maksimum servo açısı

                int8_t solAngle = 0; // Sol elevon açısı (derece cinsinden)
                int8_t sagAngle = 0; // Sağ elevon açısı (derece cinsinden)
                
            public:
                ServoPWM() {}

                void init() {
                    pinMode(ELEVON_SOL_PIN, OUTPUT);
                    pinMode(ELEVON_SAG_PIN, OUTPUT);
                    disarm();
                }

                void setPosition(float solAngle, float sagAngle) {

                    // Gelen açıları sınırla ve 0-255 aralığına dönüştür
                    uint8_t solAci = (constrain(solAngle, SERVO_MIN, SERVO_MAX) - SERVO_MIN) * 255 / (SERVO_MAX - SERVO_MIN);
                    uint8_t sagAci = (constrain(sagAngle, SERVO_MIN, SERVO_MAX) - SERVO_MIN) * 255 / (SERVO_MAX - SERVO_MIN);

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