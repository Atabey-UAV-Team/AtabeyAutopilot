#pragma once

namespace atabey {
    namespace core {

        class ParameterStore {
        public:
            void load();
            void save();
            float get(const char* name);
            void set(const char* name, float value);
        };

    }
}
