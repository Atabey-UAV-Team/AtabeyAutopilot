#pragma once
#include <Arduino.h>

namespace atabey {
    namespace drivers {

        class IServoDriver {
            public:
                virtual void init() = 0;
                virtual void write_us(uint8_t channel, uint16_t us) = 0;
        };

        class Timer4ServoDriver : public IServoDriver {
            public:
                void init() override {

                    // PIN AYARI
                    pinMode(6, OUTPUT); // OC4A
                    pinMode(7, OUTPUT); // OC4B
                    pinMode(8, OUTPUT); // OC4C

                    // PWM Mode (Fast PWM, ICR4 top)
                    TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM41);
                    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41); // prescaler 8

                    // 20 ms period
                    ICR4 = 40000;

                    // başlangıçta disarm
                    OCR4A = 2000; // 2 ms
                    OCR4B = 2000; // 2 ms
                    OCR4C = 2000; // 2 ms

                }

                void write_us(uint8_t channel, uint16_t us) override {

                    // güvenlik clamp
                    if (us < 1000) us = 1000;
                    if (us > 2000) us = 2000;

                    uint16_t ticks = us * 2;

                    switch(channel) {
                        case 0: OCR4A = ticks; break;
                        case 1: OCR4B = ticks; break;
                        case 2: OCR4C = ticks; break;
                        default: break; // geçersiz kanal, görmezden gel
                    }
                }
        };

    }
}