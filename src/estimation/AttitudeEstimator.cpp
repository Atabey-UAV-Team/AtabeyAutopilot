#include "AttitudeEstimator.h"
#include <math.h>

#define ALPHA 0.02f

namespace atabey {
    namespace estimation {

        AttitudeEstimator::AttitudeEstimator(atabey::drivers::ImuSensor& imuSensor) : imu(&imuSensor) {}

        bool AttitudeEstimator::init() {
            roll = 0, pitch = 0, yaw = 0;
            pitchAcc = 0, rollAcc = 0;
            return true;
        }

        void AttitudeEstimator::update() {
            imu->update();
            if (!imu->isHealthy()) return;

            atabey::utils::Vec3f accel = imu->getAccel();
            atabey::utils::Vec3f gyro = imu->getGyro();

            static unsigned long prevMicros = micros();
            unsigned long nowMicros = micros();

            float dt = (nowMicros - prevMicros) / 1000000.0f; // Saniyeye dönüştürmek için
            prevMicros = nowMicros;
            if (dt <= 0.0f) { dt = 0.01f; }
            else if (dt > 0.1f) { dt = 0.01f; }

            pitchAcc = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));
            rollAcc  = atan2f(accel.y, accel.z);

            roll = atabey::utils::lerp(rollAcc, roll + gyro.x * dt, ALPHA);
            pitch = atabey::utils::lerp(pitchAcc, pitch + gyro.y * dt, ALPHA);
            yaw   += gyro.z * dt;

            // Açıları -180..180 aralığında tut
            roll  = atabey::utils::wrapPi(roll);
            pitch = atabey::utils::wrapPi(pitch);
            yaw   = atabey::utils::wrapPi(yaw);
        }

        atabey::utils::Vec3f AttitudeEstimator::getAttitude() const {
            return {roll, pitch, yaw};
        }

    }
}