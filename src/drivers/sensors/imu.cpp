#include <Arduino.h>
#include "imu.h"
#include <Wire.h>
#include "../../utils/MathUtils.h"

using namespace atabey::utils;

#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C
#define INT_PIN_CFG 0x37
#define AK8963_CNTL 0x0A

#define PWR_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43
#define ACCEL_XOUT_H 0x3B
#define MAG_XOUT_L 0x03
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

namespace atabey {
    namespace drivers {

        ImuSensor::ImuSensor() : ax(0), ay(0), az(0), gx(0), gy(0), gz(0), mx(0), my(0), mz(0), healthy(false) {}

        bool ImuSensor::init() {
            Wire.begin();
            Wire.setClock(400000);
            healthy = true;

            writeRegister(MPU9250_ADDR, PWR_MGMT_1, 0x00); // Uyku modunu kapat
            writeRegister(MPU9250_ADDR, GYRO_CONFIG, 0x00); // ±250°/s
            writeRegister(MPU9250_ADDR, ACCEL_CONFIG, 0x00); // ±2g
            writeRegister(MPU9250_ADDR, INT_PIN_CFG, 0x02); // Bypass modunu açarak AK8963'e doğrudan erişim sağla
            writeRegister(AK8963_ADDR, AK8963_CNTL, 0x16); // AK8963'ü 16-bit, continuous measurement mode 2 (100Hz) moduna al

            return healthy;
        }

        void ImuSensor::update() {
            uint8_t buf[14];

            // Akselometre
            if (!readBytes(MPU9250_ADDR, ACCEL_XOUT_H, buf, 14)) return;

            ax = (int16_t)(buf[0] << 8 | buf[1]) / 16384.0f;
            ay = (int16_t)(buf[2] << 8 | buf[3]) / 16384.0f;
            az = (int16_t)(buf[4] << 8 | buf[5]) / 16384.0f;

            // Jiroskop
            gx = deg2rad((int16_t)(buf[8]  << 8 | buf[9])  / 131.0f);
            gy = deg2rad((int16_t)(buf[10] << 8 | buf[11]) / 131.0f);
            gz = deg2rad((int16_t)(buf[12] << 8 | buf[13]) / 131.0f);

            // Manyetometre
            if (!readBytes(AK8963_ADDR, MAG_XOUT_L, buf, 6)) return;

            mx = (int16_t)(buf[1] << 8 | buf[0]) / 10.0f;
            my = (int16_t)(buf[3] << 8 | buf[2]) / 10.0f;
            mz = (int16_t)(buf[5] << 8 | buf[4]) / 10.0f;

            healthy = true;
        }

        bool ImuSensor::isHealthy() const {
            return healthy;
        }

        bool ImuSensor::writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            Wire.write(data);

            bool ok = (Wire.endTransmission() == 0);
            return ok;
        }

        bool ImuSensor::readBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len) {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0) {
                healthy = false;
                return healthy;
            }

            Wire.requestFrom(addr, len, 1);
            if (Wire.available() < len) {
                healthy = false;
                return healthy;
            }

            for (uint8_t i = 0; i < len && Wire.available(); i++) {
                buffer[i] = Wire.read();
            }
            healthy = true;
            return healthy;
        }

        Vec3f ImuSensor::getAccel() const {
            return Vec3f(ax, ay, az);
        }

        Vec3f ImuSensor::getGyro() const {
            return Vec3f(gx, gy, gz);
        }

        Vec3f ImuSensor::getMag() const {
            return Vec3f(mx, my, mz); 
        }
        
   }
}