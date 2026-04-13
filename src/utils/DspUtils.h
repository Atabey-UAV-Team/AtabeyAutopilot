#pragma once
#include "Arduino.h"

namespace atabey {
    namespace utils {

        class lowpass {
        private:
            float prev;
            float current;
            float alpha;
        public:
            lowpass(float alpha_) : prev(0.0f), current(0.0f), alpha(alpha_) {}

            float update(float input) {
                current = alpha * input + (1.0f - alpha) * prev;
                prev = current;
                return current;
            }

            void reset(float value = 0.0f) {
                prev = value;
                current = value;
            };
        };

        template<int N>
        struct MovingAverage {
            float buffer[N]{0.0f};
            int index{0};
            float sum{0.0f};

            float update(float input) {
                sum -= buffer[index];
                buffer[index] = input;
                sum += input;
                index = (index + 1) % N;
                return sum / N;
            };
        };

        inline float normalizeSignal(int value, int minVal, int maxVal) {
            return (float)(value - minVal) / (maxVal - minVal) * 2.0f - 1.0f;
        }

        inline float normalizeThrottle(int value, int minVal, int maxVal) {
            return (float)(value - minVal) / (maxVal - minVal);
        }

    }
}