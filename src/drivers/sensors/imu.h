#pragma once

#include "Arduino.h"
#include "ISensor.h"
#include "../../utils/MathUtils.h"

using namespace atabey::utils;
using namespace atabey::drivers;

namespace atabey {
    namespace drivers {

        class ImuSensor : public ISensor {
            private:
                float ax, ay, az; // Akselometre data
                float gx, gy, gz; // Jiroskop data
                float mx, my, mz; // Manyetometre data

                float gyroBiasX, gyroBiasY, gyroBiasZ; // Jiroskop bias'ı (kalibrasyon sonrası)

                bool healthy;
            public:
                ImuSensor();

                bool init() override;
                void update() override;
                bool calibrate();

                bool isHealthy() const override;
                
                bool writeRegister(uint8_t addr, uint8_t reg, uint8_t data);
                bool readBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len);

                Vec3f getAccel() const;
                Vec3f getGyro() const;
                Vec3f getMag() const;
        };
    
    }
}