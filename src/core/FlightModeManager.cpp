#include "FlightModeManager.h"

namespace atabey {
    namespace core {

        FlightModeManager::FlightModeManager() : currentMode(FlightMode::MANUAL) {}

        void FlightModeManager::setMode(FlightMode mode) {
            currentMode = mode;
        }

        FlightMode FlightModeManager::getMode() const {
            return currentMode;
        }

    }
}
