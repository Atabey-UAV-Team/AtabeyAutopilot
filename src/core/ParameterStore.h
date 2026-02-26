#pragma once

namespace atabey {
    namespace core {

        class ParameterStore {
        public:
            ParameterStore();
            void load();
            void save();
            float get(const char* name) const;
            void set(const char* name, float value);
        };

    }
}
