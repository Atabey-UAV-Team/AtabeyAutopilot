#pragma once

namespace atabey {
    namespace core {

        class HealthMonitor {
        private:
            bool healthy;

        public:
            HealthMonitor();
            void update();
            bool isHealthy() const;
        };

    }
}
