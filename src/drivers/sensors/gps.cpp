#include "GpsSensor.h"

#define GPS_SERIAL Serial1
#define GPS_BAUDRATE 38400

#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07

namespace atabey {
    namespace drivers {

        GpsSensor::GpsSensor() {
            state = WAIT_SYNC1;
        }

        bool GpsSensor::init() {
            GPS_SERIAL.begin(GPS_BAUDRATE);
            state = WAIT_SYNC1;
            return true;
        }

        void GpsSensor::update() {
            while (GPS_SERIAL.available()) {
                uint8_t c = GPS_SERIAL.read();
                parseByte(c);
            }

        }

        void GpsSensor::parseByte(uint8_t c) {
            switch (state) {
                case WAIT_SYNC1:
                    if (c == 0xB5)
                        state = WAIT_SYNC2;

                    break;

                case WAIT_SYNC2:
                    if (c == 0x62)
                        state = READ_CLASS;
                    else
                        state = WAIT_SYNC1;

                    break;

                case READ_CLASS:
                    msgClass = c;
                    ckA = ckB = 0;
                    ckA += c;
                    ckB += ckA;
                    state = READ_ID;

                    break;

                case READ_ID:
                    msgId = c;
                    ckA += c;
                    ckB += ckA;
                    state = READ_LENGTH1;

                    break;

                case READ_LENGTH1:
                    length = c;
                    ckA += c;
                    ckB += ckA;
                    state = READ_LENGTH2;

                    break;

                case READ_LENGTH2:
                    length |= (uint16_t)c << 8;

                    ckA += c;
                    ckB += ckA;

                    counter = 0;
                    if (length > sizeof(payload)) {

                        state = WAIT_SYNC1;
                        break;

                    }
                    state = READ_PAYLOAD;

                    break;

                case READ_PAYLOAD:
                    payload[counter++] = c;
                    ckA += c;
                    ckB += ckA;

                    if (counter >= length)
                        state = READ_CKA;

                    break;

                case READ_CKA:
                    if (c == ckA)
                        state = READ_CKB;
                    else
                        state = WAIT_SYNC1;

                    break;

                case READ_CKB:
                    if (c == ckB)
                        processPacket();

                    state = WAIT_SYNC1;

                    break;
            }
        }

        void GpsSensor::processPacket() {
            if (msgClass == UBX_CLASS_NAV && msgId == UBX_ID_NAV_PVT) {

                if (length != sizeof(NavPVT))
                    return;
                    
                NavPVT* nav = reinterpret_cast<NavPVT*>(payload);

                gps.lat = nav->lat;
                gps.lon = nav->lon;
                gps.alt = nav->hMSL;

                gps.velN = nav->velN;
                gps.velE = nav->velE;
                gps.velD = nav->velD;

                gps.fixType = nav->fixType;

                gps.lastUpdate = millis();
            }
        }

        bool GpsSensor::isHealthy() const {
            // GPS verisi en fazla 1 saniye eski olabilir
            return millis() - gps.lastUpdate < 2000;
        }

        bool GpsSensor::hasFix() {
            return gps.fixType >= 3;
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