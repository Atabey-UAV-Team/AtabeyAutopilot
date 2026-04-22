#pragma once

#include "IEstimator.h"
#include "../drivers/sensors/imu.h"
#include "../utils/MathUtils.h"

using namespace atabey::drivers;
using namespace atabey::utils;

namespace atabey {
    namespace estimation {

        class AttitudeEstimator : public IEstimator {
            private:
                ImuSensor* imu;

                float roll{0};
                float pitch{0};
                float yaw{0};

                float pitchAcc{0};
                float rollAcc{0};

                float rollBias{0};
                float pitchBias{0};
                float yawBias{0};
                float pitchOffset = 0.0f; 
                float rollOffset  = 0.0f;
                float P_roll[2][2];
                float P_pitch[2][2];
                float P_yaw[2][2];

                float magDeclination{0.0f};

                float Q_angle      = 0.001f;
                float Q_bias       = 0.003f;
                float R_measure    = 0.03f;
                float R_measure_yaw = 0.3f;

                float dt{0.01f};
                uint32_t prevMicros{0};
                uint32_t nowMicros{0};

            public:
                AttitudeEstimator(ImuSensor& imuSensor);

                bool init() override;
                void update() override;
                void setMagDeclination(float decl) { magDeclination = decl; }  
                Vec3f getAttitude() const;
                Vec3f getRates() const;
        };

    }
}