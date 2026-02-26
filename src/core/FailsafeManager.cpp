#include "FailsafeManager.h"

namespace atabey {
    namespace core {

        FailsafeManager::FailsafeManager() : active(false), lastReason(FailsafeReason::NONE) {}

        void FailsafeManager::check() {
            // TODO: HealthMonitor + sensör durumları kontrol edilecek
        }

        void FailsafeManager::trigger(FailsafeReason reason) {
            active = true;
            lastReason = reason;
            // TODO: RTL, motor kesme vs.
        }

        bool FailsafeManager::isActive() const {
            return active;
        }

    }
}
