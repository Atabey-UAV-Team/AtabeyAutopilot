#include "HealthMonitor.h"

namespace atabey {
    namespace core {

        void HealthMonitor::update() {
            // TODO: sensör, link, batarya kontrolü yapacak
        }

        bool HealthMonitor::isHealthy() const {
            return true; // şimdilik sağlıklı kabul ediyoruz :D
        }

    }
}
