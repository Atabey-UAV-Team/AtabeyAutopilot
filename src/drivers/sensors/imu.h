#pragma once

#include "Arduino.h"
#include "../ISensor.h"

namespace atabey {
    namespace drivers {

        class ImuDriver : public atabey::drivers::ISensor {
        public:
            ImuDriver();

            bool init() override;
            void update() override;
            bool isHealthy() const override;
        };

    }
}