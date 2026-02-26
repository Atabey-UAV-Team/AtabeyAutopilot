#include "HealthMonitor.h"

namespace atabey {
    namespace core {

        HealthMonitor::HealthMonitor() : healthy(true) {}

        void HealthMonitor::update() {
            // TODO: sensör, link, batarya kontrolü yapacak
        }

        bool HealthMonitor::isHealthy() const {
            return healthy;
        }

    }
}
