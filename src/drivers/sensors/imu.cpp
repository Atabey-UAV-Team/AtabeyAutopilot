#include <Arduino.h>
#include "imu.h"
#include <Wire.h>
#include "../../utils/MathUtils.h"

using namespace atabey::utils;

#define MPU9250_ADDR  0x68
#define AK8963_ADDR   0x0C
#define INT_PIN_CFG   0x37
#define AK8963_CNTL   0x0A

#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define MAG_XOUT_L    0x03
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C

// Hard-Iron Katsayılar
#define MAG_OFFSET_X   52.0f
#define MAG_OFFSET_Y   154.0f
#define MAG_OFFSET_Z   2.5f

#define MAG_SCALE_X    1.100f
#define MAG_SCALE_Y    1.126f
#define MAG_SCALE_Z    0.831f

#define CALIBRATION_SAMPLES 500
#define GYRO_STABLE_THRESHOLD 0.1f // rad/s cinsinden

namespace atabey {
    namespace drivers {

        ImuSensor::ImuSensor()
            : ax(0), ay(0), az(0),
              gx(0), gy(0), gz(0),
              mx(0), my(0), mz(0),
              healthy(false) {}

        bool ImuSensor::init() {
            Wire.begin();
            Wire.setClock(400000); // 400kHz clock frekansı

            if (!writeRegister(MPU9250_ADDR, PWR_MGMT_1, 0x00)) { // Uyku modunu kapat
                healthy = false;
                return false;
            }
            delay(100);

            // 2) Jiroskop ±250 deg/s, akselometre ±2g
            writeRegister(MPU9250_ADDR, GYRO_CONFIG,  0x00); // ±250°/s
            writeRegister(MPU9250_ADDR, ACCEL_CONFIG, 0x00); // ±2g
            writeRegister(MPU9250_ADDR, INT_PIN_CFG, 0x02); // Bypass modunu açarak AK8963'e doğrudan erişim sağla
            delay(10);

            writeRegister(AK8963_ADDR, AK8963_CNTL, 0x00);
            delay(10);
            writeRegister(AK8963_ADDR, AK8963_CNTL, 0x16); // AK8963'ü 16-bit, continuous measurement mode 2 (100Hz) moduna al
            delay(10);

            healthy = true;
            return true;
        }

        void ImuSensor::update() {
            uint8_t buf[14];

            if (!readBytes(MPU9250_ADDR, ACCEL_XOUT_H, buf, 14)) return;

            ax = (int16_t)(buf[0] << 8 | buf[1]) / 16384.0f;
            ay = (int16_t)(buf[2] << 8 | buf[3]) / 16384.0f;
            az = (int16_t)(buf[4] << 8 | buf[5]) / 16384.0f;
            gx = deg2rad((int16_t)(buf[8]  << 8 | buf[9])  / 131.0f) - gyroBiasX; 
            gy = deg2rad((int16_t)(buf[10] << 8 | buf[11]) / 131.0f) - gyroBiasY;
            gz = deg2rad((int16_t)(buf[12] << 8 | buf[13]) / 131.0f) - gyroBiasZ;

            // Manyetometre 
            uint8_t mbuf[7];
            if (!readBytes(AK8963_ADDR, MAG_XOUT_L, mbuf, 7)) return;

            // ST2 bit3 = HOFL (overflow) — taşma varsa bu döngüyü atla
            if (mbuf[6] & 0x08) return;

            // AK8963 little-endian + hard-iron offset
            mx = ((int16_t)(mbuf[1] << 8 | mbuf[0]) - MAG_OFFSET_X) * MAG_SCALE_X;
            my = ((int16_t)(mbuf[3] << 8 | mbuf[2]) - MAG_OFFSET_Y) * MAG_SCALE_Y;
            mz = ((int16_t)(mbuf[5] << 8 | mbuf[4]) - MAG_OFFSET_Z) * MAG_SCALE_Z;

            healthy = true;

        }

        bool ImuSensor::calibrate() {
            Vec3f gyroSum(0, 0, 0);
            uint16_t count = 0;

            while (count < CALIBRATION_SAMPLES) {  // define hatasını da bypass ettim
                update();

                gyroSum.x += gx;
                gyroSum.y += gy;
                gyroSum.z += gz;

                count++;
                delay(10);
            }

            gyroBiasX = gyroSum.x / count;
            gyroBiasY = gyroSum.y / count;
            gyroBiasZ = gyroSum.z / count;

            return true;
        }

        bool ImuSensor::isHealthy() const { return healthy; }

        bool ImuSensor::writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            Wire.write(data);
            return (Wire.endTransmission() == 0);
        }

        bool ImuSensor::readBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len) {
            Wire.beginTransmission(addr);
            Wire.write(reg);

            if (Wire.endTransmission(false) != 0) { 
                healthy = false; return false; 
            }

            Wire.requestFrom((uint8_t)addr, (uint8_t)len, (uint8_t)1);

            if (Wire.available() < len) { 
                healthy = false; return false; 
            }

            for (uint8_t i = 0; i < len; i++) {
                buffer[i] = Wire.read();
            }

            return true;
        }

        Vec3f ImuSensor::getAccel() const {
            if (!healthy) return {0.0f, 0.0f, 1.0f};
            return {ax, ay, az};
        }
        Vec3f ImuSensor::getGyro() const { return {gx, gy, gz}; }
        Vec3f ImuSensor::getMag()  const { return {mx, my, mz}; }

    }
}