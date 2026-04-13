#include "AttitudeEstimator.h"
#include <math.h>

#define ALPHA 0.98f

using namespace atabey::utils;

namespace atabey {
    namespace estimation {

        AttitudeEstimator::AttitudeEstimator(atabey::drivers::ImuSensor& imuSensor) : imu(&imuSensor) {}

        bool AttitudeEstimator::init() {
            roll = pitch = yaw = 0.0f;
            pitchAcc = rollAcc = 0.0f;
            rollBias = pitchBias = 0.0f;
            dt = 0.01f; // Başlangıçta 10ms varsayıyoruz

            P_roll[0][0]  = 1.0f; P_roll[0][1]  = 0.0f;
            P_roll[1][0]  = 0.0f; P_roll[1][1]  = 1.0f;

            P_pitch[0][0] = 1.0f; P_pitch[0][1] = 0.0f;
            P_pitch[1][0] = 0.0f; P_pitch[1][1] = 1.0f;

            prevMicros = micros();
            return true;
        }

        void AttitudeEstimator::update(){

            ImuSample sample;
            sample.accel = imu->getAccel();
            sample.gyro = imu->getGyro();
            sample.mag = imu->getMag();

            nowMicros = micros();
            dt = (nowMicros - prevMicros) / 1000000.0f; // Saniyeye dönüştürmek için
            prevMicros = nowMicros;

            if (dt <= 0.0f) { dt = 0.01f; } // dt'nin sıfır veya negatif gelmesi durumunu engelledik
            else if (dt > 0.1f) { dt = 0.1f; }

            pitchAcc = atan2f(-sample.accel.x, sqrtf(sample.accel.y * sample.accel.y + sample.accel.z * sample.accel.z)); // Pitch açısını hesaplamak için akselometre verilerini kullandık
            rollAcc  = atan2f(sample.accel.y, sample.accel.z); // Roll açısını hesaplamak için akselometre verilerini kullandık

            // ===== ROLL KALMAN =====
            float rollRate = sample.gyro.x - rollBias;
            roll += dt * rollRate;

            P_roll[0][0] += dt * (dt*P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
            P_roll[0][1] -= dt * P_roll[1][1];
            P_roll[1][0] -= dt * P_roll[1][1];
            P_roll[1][1] += Q_bias * dt;

            float S = P_roll[0][0] + R_measure;
            float K0 = P_roll[0][0] / S;
            float K1 = P_roll[1][0] / S;

            float y = rollAcc - roll;

            roll += K0 * y;
            rollBias += K1 * y;

            float P00_temp = P_roll[0][0];
            float P01_temp = P_roll[0][1];

            P_roll[0][0] -= K0 * P00_temp;
            P_roll[0][1] -= K0 * P01_temp;
            P_roll[1][0] -= K1 * P00_temp;
            P_roll[1][1] -= K1 * P01_temp;

            // ===== PITCH KALMAN =====
            float pitchRate = sample.gyro.y - pitchBias;
            pitch += dt * pitchRate;

            P_pitch[0][0] += dt * (dt*P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
            P_pitch[0][1] -= dt * P_pitch[1][1];
            P_pitch[1][0] -= dt * P_pitch[1][1];
            P_pitch[1][1] += Q_bias * dt;

            S = P_pitch[0][0] + R_measure;
            K0 = P_pitch[0][0] / S;
            K1 = P_pitch[1][0] / S;

            y = pitchAcc - pitch;

            pitch += K0 * y;
            pitchBias += K1 * y;

            P00_temp = P_pitch[0][0];
            P01_temp = P_pitch[0][1];

            P_pitch[0][0] -= K0 * P00_temp;
            P_pitch[0][1] -= K0 * P01_temp;
            P_pitch[1][0] -= K1 * P00_temp;
            P_pitch[1][1] -= K1 * P01_temp;

            // ===== YAW KALMAN =====
            Vec3f mag = imu->getMag();

            float magX = mag.x * cos(pitch) + mag.z * sin(pitch);
            float magY = mag.x * sin(roll)*sin(pitch) + mag.y*cos(roll) - mag.z*sin(roll)*cos(pitch);

            float yawMag = atan2f(-magY, magX);

            yaw = lerp(yawMag, yaw + sample.gyro.z * dt, 0.98f);

            // Açıları -180..180 aralığında tutar
            roll  = wrapPi(roll);
            pitch = wrapPi(pitch);
            yaw   = wrapPi(yaw);
        }

        Vec3f AttitudeEstimator::getAttitude() const {
            return {roll, pitch, yaw};
        }

    }
}