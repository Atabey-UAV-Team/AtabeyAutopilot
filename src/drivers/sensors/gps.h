#pragma once

#include <Arduino.h>
#include "ISensor.h"

#define BUFFER_SIZE 128

namespace atabey {
    namespace drivers {

        struct GpsState {
            int32_t lat = 0; // deg * 1e7
            int32_t lon = 0; // deg * 1e7
            int32_t alt = 0; // mm

            int32_t velN = 0;
            int32_t velE = 0;
            int32_t velD = 0;

            uint8_t fixType = 0;
            uint8_t satellites = 0;

            uint32_t lastUpdate = 0;
        };

        class GpsSensor : public ISensor {
            private:
                char lineBuffer[BUFFER_SIZE];
                uint8_t index = 0;

                GpsState gps;

                void parseLine();
                int32_t parseLatLon(const char* value, const char* hemi);
                float parseFloat(const char* s);

            public:
                GpsSensor();

                bool init();
                void update();

                bool hasFix();
                bool isHealthy() const;

                int32_t getLat();
                int32_t getLon();
                int32_t getAlt();
        };

    }
}