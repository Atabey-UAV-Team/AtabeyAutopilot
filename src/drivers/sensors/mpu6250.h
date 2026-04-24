#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace atabey {
    namespace drivers {

        class MPU6250 {
        public:
            MPU6250();

            bool init();
            bool calibrateGyro(uint16_t samples = 500);
            bool read();

            float ax, ay, az;
            float gx, gy, gz;

            bool healthy() const { return healthy_; }

        private:
            float gyroBiasX_, gyroBiasY_, gyroBiasZ_;
            bool  healthy_;

            bool writeReg(uint8_t reg, uint8_t data);
            bool readBytes(uint8_t reg, uint8_t* buf, uint8_t len);
        };

    }
}
