#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace atabey {
    namespace core {

        class Scheduler {
        private:
            uint32_t lastTickMs;

        public:
            Scheduler();
            void tick();
        };

    }
}
