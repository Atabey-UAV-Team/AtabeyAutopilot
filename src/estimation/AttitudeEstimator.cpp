#include "AttitudeEstimator.h"
#include <math.h>

using namespace atabey::utils;

namespace atabey {
    namespace estimation {

        AttitudeEstimator::AttitudeEstimator(atabey::drivers::ImuSensor& imuSensor)
            : imu(&imuSensor) {}

        bool AttitudeEstimator::init() {
            roll = pitch = yaw = 0.0f;
            pitchAcc = rollAcc = 0.0f;
            rollBias = pitchBias = yawBias = 0.0f;
            pitchOffset = rollOffset = 0.0f;
            dt = 0.01f;

            P_roll[0][0]  = 1.0f; P_roll[0][1]  = 0.0f;
            P_roll[1][0]  = 0.0f; P_roll[1][1]  = 1.0f;

            P_pitch[0][0] = 1.0f; P_pitch[0][1] = 0.0f;
            P_pitch[1][0] = 0.0f; P_pitch[1][1] = 1.0f;

            P_yaw[0][0]   = 1.0f; P_yaw[0][1]   = 0.0f;
            P_yaw[1][0]   = 0.0f; P_yaw[1][1]   = 1.0f;

            // ── Akselometre offset otomatik kalibrasyon ──────────────
            float sumPitch = 0.0f, sumRoll = 0.0f;
            const int N = 100;
            for (int i = 0; i < N; i++) {
                imu->update();
                Vec3f a = imu->getAccel();
                sumPitch += atan2f(-a.x, sqrtf(a.y*a.y + a.z*a.z));
                sumRoll  += atan2f(a.y, a.z);
                delay(5);
            }
            pitchOffset = sumPitch / N;
            rollOffset  = sumRoll  / N;

            // ── Yaw'ı manyetometreden initialize et ─────────────────
            imu->update();
            Vec3f mag0 = imu->getMag();
            float magX0 =  mag0.x *  cosf(pitch)
                         + mag0.y *  sinf(pitch) * sinf(roll)
                         - mag0.z *  sinf(pitch) * cosf(roll);
            float magY0 =  mag0.y *  cosf(roll)
                         + mag0.z *  sinf(roll);
            yaw = atan2f(-magY0, magX0);

            prevMicros = micros();
            return true;
        }

        void AttitudeEstimator::update() {
            
            nowMicros = micros();
            dt = (nowMicros - prevMicros) / 1000000.0f;
            prevMicros = nowMicros;
            if (dt <= 0.0f)    dt = 0.01f;
            else if (dt > 0.1f) dt = 0.1f;

            // ─── Sensör verisi ──────
            imu->update();                     
            Vec3f accel = imu->getAccel();
            Vec3f gyro  = imu->getGyro();
            Vec3f mag   = imu->getMag();

            // ─── Akselometre referans (offset düzeltilmiş) ──────────
            pitchAcc = atan2f(-accel.x, sqrtf(accel.y*accel.y + accel.z*accel.z)) - pitchOffset;
            rollAcc  = atan2f(accel.y, accel.z) - rollOffset;

            // ===== ROLL KALMAN ============
            {
                roll += dt * (gyro.x - rollBias);

                P_roll[0][0] += dt * (dt*P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
                P_roll[0][1] -= dt * P_roll[1][1];
                P_roll[1][0] -= dt * P_roll[1][1];
                P_roll[1][1] += Q_bias * dt;

                float S  = P_roll[0][0] + R_measure;
                float K0 = P_roll[0][0] / S;
                float K1 = P_roll[1][0] / S;
                float inn = rollAcc - roll;

                roll     += K0 * inn;
                rollBias += K1 * inn;

                float P00 = P_roll[0][0], P01 = P_roll[0][1];
                P_roll[0][0] -= K0 * P00;
                P_roll[0][1] -= K0 * P01;
                P_roll[1][0] -= K1 * P00;
                P_roll[1][1] -= K1 * P01;
            }
            // ===== PITCH KALMAN ==========
            {
                pitch += dt * (gyro.y - pitchBias);

                P_pitch[0][0] += dt * (dt*P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
                P_pitch[0][1] -= dt * P_pitch[1][1];
                P_pitch[1][0] -= dt * P_pitch[1][1];
                P_pitch[1][1] += Q_bias * dt;

                float S  = P_pitch[0][0] + R_measure;
                float K0 = P_pitch[0][0] / S;
                float K1 = P_pitch[1][0] / S;
                float inn = pitchAcc - pitch;

                pitch     += K0 * inn;
                pitchBias += K1 * inn;

                float P00 = P_pitch[0][0], P01 = P_pitch[0][1];
                P_pitch[0][0] -= K0 * P00;
                P_pitch[0][1] -= K0 * P01;
                P_pitch[1][0] -= K1 * P00;
                P_pitch[1][1] -= K1 * P01;
            }

            // ===== YAW KALMAN =========
            
            float mx =  mag.y;   // ← dikkat: y→x swap
            float my =  mag.x;   // ← dikkat: x→y swap  
            float mz = -mag.z;   // ← z ters

            float magX =  mx * cosf(pitch)
                        + my * sinf(pitch) * sinf(roll)
                        + mz * cosf(pitch) * cosf(roll);   // işaret düzeltildi

            float magY =  my * cosf(roll)
                        - mz * sinf(roll);                 // işaret düzeltildi

            float yawMag = atan2f(-magY, magX);

            yaw += dt * (gyro.z - yawBias);

            P_yaw[0][0] += dt * (dt*P_yaw[1][1] - P_yaw[0][1] - P_yaw[1][0] + Q_angle);
            P_yaw[0][1] -= dt * P_yaw[1][1];
            P_yaw[1][0] -= dt * P_yaw[1][1];
            P_yaw[1][1] += Q_bias * dt;

            // Update
            {
                float inn   = wrapPi(yawMag - yaw);
                float S_yaw = P_yaw[0][0] + R_measure_yaw;
                float Ky0   = P_yaw[0][0] / S_yaw;
                float Ky1   = P_yaw[1][0] / S_yaw;

                yaw     += Ky0 * inn;
                yawBias += Ky1 * inn;

                float P00 = P_yaw[0][0], P01 = P_yaw[0][1];
                P_yaw[0][0] -= Ky0 * P00;
                P_yaw[0][1] -= Ky0 * P01;
                P_yaw[1][0] -= Ky1 * P00;
                P_yaw[1][1] -= Ky1 * P01;
            }

            roll  = wrapPi(roll);
            pitch = wrapPi(pitch);
            yaw = wrapPi(yaw + magDeclination);
        }

        Vec3f AttitudeEstimator::getAttitude() const { return {roll, pitch, yaw}; }
        Vec3f AttitudeEstimator::getRates()    const { return {rollBias, pitchBias, yawBias}; }

    }
}