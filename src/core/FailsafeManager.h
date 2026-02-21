#pragma once
#include "FailsafeReason.h"

namespace atabey {
    namespace core {

        class FailsafeManager {
        public:
            void check();
            void trigger(FailsafeReason reason);
        };

    }
}
