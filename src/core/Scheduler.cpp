#include "Scheduler.h"

namespace atabey {
    namespace core {

        Scheduler::Scheduler() : lastTickMs(0) {}

        void Scheduler::tick() {
            lastTickMs = millis();
            // TODO: Periyodik görevler eklenecek
            // Örneğin, 10 Hz Imu verisi, 50 Hz PID döngüsü 
        }

    }
}
