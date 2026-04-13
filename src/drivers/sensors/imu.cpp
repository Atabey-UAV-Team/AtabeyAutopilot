#include <Arduino.h>
#include "imu.h"
#include <Wire.h>
#include "../../utils/MathUtils.h"
#include <math.h>

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

// Kalibrasyon parametreleri
#define CALIBRATION_SAMPLES 1000
#define GYRO_STABLE_THRESHOLD 0.1f // Jiroskopun stabil kabul edileceği eşik değeri (rad/s)

namespace atabey {
    namespace drivers {

        ImuSensor::ImuSensor() : accel(0, 0, 0), gyro(0, 0, 0), mag(0, 0, 0), gyroBias(0, 0, 0), healthy(false) {}

        bool ImuSensor::init() {
            Wire.begin();
            Wire.setClock(400000); // 400kHz clock frekansı
            healthy = isHealthy();

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

            accel.x = (int16_t)(buf[0] << 8 | buf[1]) / 1670.7f; // ±2g için 16384 LSB/g yani 2^15 / 4
            accel.y = (int16_t)(buf[2] << 8 | buf[3]) / 1670.7f; // 16384.0f / 9.80665f = 1670,7f LSB/(m/s²)
            accel.z = (int16_t)(buf[4] << 8 | buf[5]) / 1670.7f;

            // Jiroskop
            gyro.x = deg2rad((int16_t)(buf[8]  << 8 | buf[9])  / 131.0f); // ±250°/s için 131 LSB/°/s yani 2^15 / 250
            gyro.y = deg2rad((int16_t)(buf[10] << 8 | buf[11]) / 131.0f);
            gyro.z = deg2rad((int16_t)(buf[12] << 8 | buf[13]) / 131.0f);

            gyro.x -= gyroBias.x; // Kalibrasyon bias'ını uygula
            gyro.y -= gyroBias.y;
            gyro.z -= gyroBias.z;

            // Manyetometre
            if (!readBytes(AK8963_ADDR, MAG_XOUT_L, buf, 6)) return;

            mag.x = (int16_t)(buf[1] << 8 | buf[0]) * 0.15f; // AK8963'ün manyetik alan ölçümleri 0.15µT/LSB yani 15µT/100 LSB
            mag.y = (int16_t)(buf[3] << 8 | buf[2]) * 0.15f;
            mag.z = (int16_t)(buf[5] << 8 | buf[4]) * 0.15f;

            healthy = isHealthy(); // Verilerin güncellenmesi sonrası sağlık durumunu kontrol et
        }

        bool ImuSensor::calibrate() {
            uint16_t count = 0;
            while (count < CALIBRATION_SAMPLES) {
                update();
                if (isStable()) {
                    gyroCalibration.x += gyro.x;
                    gyroCalibration.y += gyro.y;
                    gyroCalibration.z += gyro.z;
                    count++;
                } else {
                    gyroCalibration = Vec3f(0, 0, 0); // Kalibrasyon verilerini sıfırla
                    count = 0;
                }
            }

            gyroBias.x = gyroCalibration.x / CALIBRATION_SAMPLES;
            gyroBias.y = gyroCalibration.y / CALIBRATION_SAMPLES;
            gyroBias.z = gyroCalibration.z / CALIBRATION_SAMPLES;

            return true;
        }

        bool ImuSensor::isHealthy() const {
            return true; // TODO: Health Check ekle (sensör hataları, I2C iletişim sorunları vs.)
        }

        bool ImuSensor::writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            Wire.write(data);

            bool ok = (Wire.endTransmission() == 0);
            return ok;
        }

        bool ImuSensor::isStable() const {
            return (fabs(gyro.x) < GYRO_STABLE_THRESHOLD) &&
                   (fabs(gyro.y) < GYRO_STABLE_THRESHOLD) &&
                   (fabs(gyro.z) < GYRO_STABLE_THRESHOLD);
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
            return accel;
        }

        Vec3f ImuSensor::getGyro() const {
            return gyro;
        }

        Vec3f ImuSensor::getMag() const {
            return mag;
        }
        
   }
}