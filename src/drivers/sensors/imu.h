#pragma once

#include <Arduino.h>
#include "ISensor.h"
#include "../../utils/MathUtils.h"

 using namespace atabey::utils;

namespace atabey {
    namespace drivers {

        class ImuSensor : public atabey::drivers::ISensor {
            private:
                Vec3f accel = Vec3f(0, 0, 0); // Akselometre data
                Vec3f gyro = Vec3f(0, 0, 0);  // Jiroskop data
                Vec3f mag = Vec3f(0, 0, 0);   // Manyetometre data

                Vec3f gyroBias = Vec3f(0, 0, 0); // Jiroskop bias'ı (kalibrasyon sonrası)

                bool healthy;
            public:
                ImuSensor();

                bool init() override;
                void update() override;
                bool calibrate();

                bool isHealthy() const override;
                bool isStable() const; // Jiroskopun stabil olup olmadığını kontrol eder (kalibrasyon için)
                
                bool writeRegister(uint8_t addr, uint8_t reg, uint8_t data);
                bool readBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len);

                Vec3f getAccel() const;
                Vec3f getGyro() const;
                Vec3f getMag() const;
        };
    
    }
}