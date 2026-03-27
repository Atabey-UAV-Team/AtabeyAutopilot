#pragma once

namespace atabey {
    namespace core {

        enum class FailsafeReason {
            RC_LOST,
            GPS_LOST,
            LOW_BATTERY,
            SENSOR_FAILURE
        };

    }
}
