#pragma once

#include <stdint.h>
#include <stddef.h>

namespace atabey::comm {

    class ICommLink {
    public:
        virtual ~ICommLink() = default;

        virtual bool init() = 0;

        virtual bool send(const uint8_t* data, size_t len) = 0;
        virtual bool receive(uint8_t* buffer, size_t maxLen, size_t& outLen) = 0;

        virtual bool isConnected() const = 0;
    };

}