#pragma once
#include "FlightMode.h"

namespace atabey {
    namespace core {

    class FlightModeManager {
    private:
        FlightMode currentMode;

    public:
        FlightModeManager();

        void setMode(FlightMode mode);
        FlightMode getMode() const;
    };

    }
}
