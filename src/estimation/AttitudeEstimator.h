#pragma once

#include "IEstimator.h"
#include "../drivers/sensors/imu.h"
#include "../utils/MathUtils.h"

using namespace atabey::drivers;
using namespace atabey::utils;

namespace atabey {
    namespace estimation {

        struct ImuSample {
            Vec3f accel;
            Vec3f gyro;
            Vec3f mag;
        };

        class AttitudeEstimator : public IEstimator {
            private:
                ImuSensor* imu;

                float roll{0};
                float pitch{0};
                float yaw{0};

                float pitchAcc{0};
                float rollAcc{0};

                float rollBias;
                float pitchBias;

                float P_roll[2][2];
                float P_pitch[2][2];

                float Q_angle = 0.001f;
                float Q_bias  = 0.003f;
                float R_measure = 0.03f;

                float normalized{0};
                float dt;
                uint32_t prevMicros{0};
                uint32_t nowMicros{0};

                float sampleSum{0};
                float sample{0};

            public:
                AttitudeEstimator(ImuSensor& imuSensor);

                bool init() override;
                void update() override;
                Vec3f getAttitude() const;
                Vec3f getRates() const;
        };

    }
}