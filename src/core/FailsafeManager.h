#pragma once
#include "FailsafeReason.h"

namespace atabey {
    namespace core {

        class FailsafeManager {
        private:
            bool active;
            FailsafeReason lastReason;

        public:
            FailsafeManager();
            void check();
            void trigger(FailsafeReason reason);
            bool isActive() const;
        };

    }
}
