#include <Arduino.h>
#include "imu.h"
#include <Wire.h>
#include "../../utils/MathUtils.h"

using namespace atabey::utils;

#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C

#define PWR_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43
#define ACCEL_XOUT_H 0x3B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

namespace atabey {
    namespace drivers {

        ImuSensor::ImuSensor() : ax(0), ay(0), az(0), gx(0), gy(0), gz(0), mx(0), my(0), mz(0) {}

        bool ImuSensor::init() {
            Wire.begin();
            Wire.beginTransmission(MPU9250_ADDR);
            Wire.write(PWR_MGMT_1);
            Wire.write(0x00); // Wake up the MPU-9250
            return Wire.endTransmission() == 0;
        }

        void ImuSensor::update() {
            Wire.beginTransmission(MPU9250_ADDR);
            Wire.write(ACCEL_XOUT_H);
            Wire.endTransmission(false);

            Wire.requestFrom(MPU9250_ADDR, 6, true);
            ax = (Wire.read() << 8 | Wire.read()) / 16384.0f;
            ay = (Wire.read() << 8 | Wire.read()) / 16384.0f;
            az = (Wire.read() << 8 | Wire.read()) / 16384.0f;

            Wire.beginTransmission(MPU9250_ADDR);
            Wire.write(GYRO_XOUT_H);
            Wire.endTransmission(false);

            Wire.requestFrom(MPU9250_ADDR, 6, true);
            gx = (Wire.read() << 8 | Wire.read()) / 131.0f;
            gy = (Wire.read() << 8 | Wire.read()) / 131.0f;
            gz = (Wire.read() << 8 | Wire.read()) / 131.0f;
        }

        bool ImuSensor::isHealthy() const {
            return true;
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