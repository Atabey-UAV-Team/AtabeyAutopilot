#pragma once

namespace atabey::estimation {

    class IEstimator {
    public:
        virtual ~IEstimator() = default;

        virtual bool init() = 0;
        virtual void update() = 0;
    };

}