#include "gps.h"
#include <Arduino.h>

#define GPS_SERIAL Serial1
#define GPS_BAUDRATE 9600

namespace atabey {
    namespace drivers {
        GpsSensor::GpsSensor() {}

        bool GpsSensor::init() {
            GPS_SERIAL.begin(GPS_BAUDRATE);
            delay(500);
            index = 0;

            return true;
        }

        void GpsSensor::update() {

            while (GPS_SERIAL.available()) {

                char c = GPS_SERIAL.read();

                if (c == '\n') {

                    lineBuffer[index] = '\0';
                    parseLine();
                    index = 0;

                } else {

                    if (index < BUFFER_SIZE - 1)
                        lineBuffer[index++] = c;
                }
            }
        }

        int32_t GpsSensor::parseLatLon(const char* value, const char* hemi) {

            float raw = atof(value);

            int deg = (int)(raw / 100);
            float minutes = raw - deg * 100;

            float decimal = deg + minutes / 60.0;

            if (hemi[0] == 'S' || hemi[0] == 'W')
                decimal = -decimal;

            return (int32_t)(decimal * 1e7);
        }

        float GpsSensor::parseFloat(const char* s) {
            return atof(s);
        }

        void GpsSensor::parseLine() {

            if (strncmp(lineBuffer, "$GPRMC", 6) == 0) {

                char *token;
                char *ptr = lineBuffer;

                uint8_t field = 0;

                char lat[16], latH[2];
                char lon[16], lonH[2];
                char speed[16];

                while ((token = strtok_r(ptr, ",", &ptr))) {

                    switch(field) {

                        case 2: // status
                            if (token[0] != 'A')
                                return;
                            break;

                        case 3:
                            strcpy(lat, token);
                            break;

                        case 4:
                            strcpy(latH, token);
                            break;

                        case 5:
                            strcpy(lon, token);
                            break;

                        case 6:
                            strcpy(lonH, token);
                            break;

                        case 7:
                            strcpy(speed, token);
                            break;
                    }

                    field++;
                }

                gps.lat = parseLatLon(lat, latH);
                gps.lon = parseLatLon(lon, lonH);

                float knots = parseFloat(speed);
                float mps = knots * 0.514444f;

                gps.velN = (int32_t)(mps * 1000);
                gps.velE = 0;
                gps.velD = 0;

                gps.lastUpdate = millis();
            }

            else if (strncmp(lineBuffer, "$GPGGA", 6) == 0) {

                char *token;
                char *ptr = lineBuffer;

                uint8_t field = 0;

                char fix[4];
                char alt[16];

                while ((token = strtok_r(ptr, ",", &ptr))) {

                    switch(field) {

                        case 6:
                            strcpy(fix, token);
                            break;

                        case 9:
                            strcpy(alt, token);
                            break;
                    }

                    field++;
                }

                gps.fixType = atoi(fix);

                float meters = atof(alt);
                gps.alt = (int32_t)(meters * 1000);
            }
        }

        bool GpsSensor::hasFix() {
            return gps.fixType > 0;
        }

        bool GpsSensor::isHealthy() const {

            if (gps.lastUpdate == 0)
                return false;

            return millis() - gps.lastUpdate < 2000;
        }

        int32_t GpsSensor::getLat() {
            return gps.lat;
        }

        int32_t GpsSensor::getLon() {
            return gps.lon;
        }

        int32_t GpsSensor::getAlt() {
            return gps.alt;
        }

    }
}