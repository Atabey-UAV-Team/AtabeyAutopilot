#include "ParameterStore.h"

namespace atabey {
    namespace core {

        ParameterStore::ParameterStore() {}

        void ParameterStore::load() {
            // TODO: EEPROM/Flash
        }

        void ParameterStore::save() {
            // TODO: EEPROM/Flash
        }

        float ParameterStore::get(const char* name) const {
            (void)name;
            return 0.0f;
        }

        void ParameterStore::set(const char* name, float value) {
            // TODO: Tuning fonksiyonu
        }

    }
}
