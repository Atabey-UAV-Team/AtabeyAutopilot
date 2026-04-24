#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace atabey {
    namespace drivers {

        class BMM150 {
        public:
            BMM150();

            bool init();
            bool read();

            // Magnetic field in microTesla (uT), sensor frame
            float mx, my, mz;

            bool healthy() const { return healthy_; }

        private:
            bool healthy_;

            // Trim / compensation coefficients (loaded from NVM at init)
            int8_t   dig_x1, dig_y1;
            int8_t   dig_x2, dig_y2;
            uint16_t dig_z1;
            int16_t  dig_z2, dig_z3, dig_z4;
            uint8_t  dig_xy1;
            int8_t   dig_xy2;
            uint16_t dig_xyz1;

            bool writeReg(uint8_t reg, uint8_t data);
            bool readBytes(uint8_t reg, uint8_t* buf, uint8_t len);
            bool readReg(uint8_t reg, uint8_t& out);
            bool readTrim();

            float compensateX(int16_t raw, uint16_t rhall);
            float compensateY(int16_t raw, uint16_t rhall);
            float compensateZ(int16_t raw, uint16_t rhall);
        };

    }
}
