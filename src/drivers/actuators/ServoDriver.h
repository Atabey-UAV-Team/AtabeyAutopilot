#pragma once
#include <Arduino.h>

namespace atabey {
    namespace drivers {

        class IServoDriver {
            public:
                virtual void init() = 0;
                virtual void write_us(uint8_t channel, uint16_t us) = 0;
        };

        class Timer3ServoDriver : public IServoDriver {
            public:
                void init() override {

                    // PIN AYARI
                    pinMode(5, OUTPUT); // OC3A
                    pinMode(2, OUTPUT); // OC3B
                    pinMode(3, OUTPUT); // OC3C

                    // PWM Mode (Fast PWM, ICR3 top)
                    TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM31);
                    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31); // prescaler 8

                    // 20 ms period
                    ICR3 = 40000;
                }

                void write_us(uint8_t channel, uint16_t us) override {

                    // güvenlik clamp
                    if (us < 1000) us = 1000;
                    if (us > 2000) us = 2000;

                    uint16_t ticks = us * 2;

                    switch(channel) {
                        case 0: OCR3A = ticks; break;
                        case 1: OCR3B = ticks; break;
                        case 2: OCR3C = ticks; break;
                    }
                }
        };

    }
}