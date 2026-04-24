#include "mpu6250.h"
#include <Wire.h>

namespace atabey {
    namespace drivers {

        // MPU6250 (MPU-6500 compatible) register map
        static const uint8_t MPU_ADDR        = 0x68;
        static const uint8_t REG_PWR_MGMT_1  = 0x6B;
        static const uint8_t REG_SMPLRT_DIV  = 0x19;
        static const uint8_t REG_CONFIG      = 0x1A;
        static const uint8_t REG_GYRO_CONFIG = 0x1B;
        static const uint8_t REG_ACCEL_CONFIG= 0x1C;
        static const uint8_t REG_ACCEL_CFG2  = 0x1D;
        static const uint8_t REG_INT_PIN_CFG = 0x37;
        static const uint8_t REG_ACCEL_XOUT_H= 0x3B;
        static const uint8_t REG_WHO_AM_I    = 0x75;

        // ±2g, ±250 dps scaling
        static const float ACCEL_LSB_PER_G   = 16384.0f;
        static const float GYRO_LSB_PER_DPS  = 131.0f;
        static const float DEG_TO_RAD_F      = 0.01745329251f;
        static const float G_TO_MPS2         = 9.80665f;

        MPU6250::MPU6250()
            : ax(0), ay(0), az(0),
              gx(0), gy(0), gz(0),
              gyroBiasX_(0), gyroBiasY_(0), gyroBiasZ_(0),
              healthy_(false) {}

        bool MPU6250::writeReg(uint8_t reg, uint8_t data) {
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(reg);
            Wire.write(data);
            return (Wire.endTransmission() == 0);
        }

        bool MPU6250::readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0) return false;

            Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len, (uint8_t)1);
            if (Wire.available() < len) return false;

            for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
            return true;
        }

        bool MPU6250::init() {
            Wire.begin();
            Wire.setClock(400000);

            // Wake, clock = PLL auto
            if (!writeReg(REG_PWR_MGMT_1, 0x01)) { healthy_ = false; return false; }
            delay(50);

            writeReg(REG_CONFIG,       0x03); // DLPF ~41 Hz gyro
            writeReg(REG_SMPLRT_DIV,   0x00); // 1 kHz sample
            writeReg(REG_GYRO_CONFIG,  0x00); // ±250 dps
            writeReg(REG_ACCEL_CONFIG, 0x00); // ±2 g
            writeReg(REG_ACCEL_CFG2,   0x03); // DLPF ~41 Hz accel
            writeReg(REG_INT_PIN_CFG,  0x02); // I2C bypass for magnetometer
            delay(10);

            healthy_ = true;
            return true;
        }

        bool MPU6250::read() {
            uint8_t buf[14];
            if (!readBytes(REG_ACCEL_XOUT_H, buf, 14)) {
                healthy_ = false;
                return false;
            }

            int16_t rax = (int16_t)((buf[0]  << 8) | buf[1]);
            int16_t ray = (int16_t)((buf[2]  << 8) | buf[3]);
            int16_t raz = (int16_t)((buf[4]  << 8) | buf[5]);
            int16_t rgx = (int16_t)((buf[8]  << 8) | buf[9]);
            int16_t rgy = (int16_t)((buf[10] << 8) | buf[11]);
            int16_t rgz = (int16_t)((buf[12] << 8) | buf[13]);

            ax = (rax / ACCEL_LSB_PER_G) * G_TO_MPS2;
            ay = (ray / ACCEL_LSB_PER_G) * G_TO_MPS2;
            az = (raz / ACCEL_LSB_PER_G) * G_TO_MPS2;

            gx = (rgx / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F - gyroBiasX_;
            gy = (rgy / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F - gyroBiasY_;
            gz = (rgz / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F - gyroBiasZ_;

            healthy_ = true;
            return true;
        }

        bool MPU6250::calibrateGyro(uint16_t samples) {
            float sx = 0.0f, sy = 0.0f, sz = 0.0f;
            uint16_t good = 0;

            gyroBiasX_ = gyroBiasY_ = gyroBiasZ_ = 0.0f;

            for (uint16_t i = 0; i < samples; i++) {
                if (read()) {
                    sx += gx; sy += gy; sz += gz;
                    good++;
                }
                delay(2);
            }

            if (good == 0) return false;

            gyroBiasX_ = sx / (float)good;
            gyroBiasY_ = sy / (float)good;
            gyroBiasZ_ = sz / (float)good;
            return true;
        }

    }
}
