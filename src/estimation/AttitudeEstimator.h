#pragma once

#include "IEstimator.h"
#include "../drivers/sensors/imu.h"
#include "../utils/MathUtils.h"

namespace atabey {
    namespace estimation {

        class AttitudeEstimator : public IEstimator {
            private:
                atabey::drivers::ImuSensor* imu;

                float roll{0};
                float pitch{0};
                float yaw{0};

                float pitchAcc{0};
                float rollAcc{0};

            public:
                AttitudeEstimator(atabey::drivers::ImuSensor& imuSensor);

                bool init() override;
                void update() override;

                atabey::utils::Vec3f getAttitude() const;
        };

    }
}