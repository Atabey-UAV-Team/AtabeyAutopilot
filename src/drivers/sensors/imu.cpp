#include "imu.h"

namespace atabey {
    namespace drivers {

        ImuDriver::ImuDriver() {}

        bool ImuDriver::init() {
            // TODO: I2C/SPI başlatma, WHOAMI kontrolü
            return true;
        }

        void ImuDriver::update() {
            // TODO: Ham veri okunacak (ivme, jiroskop)
        }

        bool ImuDriver::isHealthy() const {
            // TODO: Sensör durumu kontrol edilecek
            return true;
        }

    }
}
