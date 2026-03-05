#pragma once

#include "Arduino.h"
#include "ISensor.h"
#include "../../utils/MathUtils.h"

namespace atabey {
    namespace drivers {

        class ImuSensor : public atabey::drivers::ISensor {
            private:
                float ax, ay, az; // Akselometre data
                float gx, gy, gz; // Jiroskop data
                float mx, my, mz; // Manyetometre data

                bool healthy;
            public:
                ImuSensor();

                bool init() override;
                void update() override;
                bool isHealthy() const override;
                bool writeRegister(uint8_t addr, uint8_t reg, uint8_t data);
                bool readBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len);

                atabey::utils::Vec3f getAccel() const;
                atabey::utils::Vec3f getGyro() const;
                atabey::utils::Vec3f getMag() const;
        };
    
    }
}