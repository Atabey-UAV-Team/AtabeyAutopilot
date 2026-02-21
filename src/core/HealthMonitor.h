#pragma once

namespace atabey {
    namespace core {

        class HealthMonitor {
        public:
            void update();
            bool isHealthy() const;
        };

    }
}
