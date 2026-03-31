#pragma once

namespace atabey {
    namespace drivers {

        template<class Driver>
        ServoPWM<Driver>::ServoPWM(Driver& driver, uint8_t ch1, uint8_t ch2)
            : _driver(driver), _ch1(ch1), _ch2(ch2) {}

        template<class Driver>
        void ServoPWM<Driver>::init() {
            _driver.init();
            disarm();
        }

        template<class Driver>
        void ServoPWM<Driver>::setPosition(float solAngle, float sagAngle) {
            _driver.write_us(_ch1, angleToUs(solAngle));
            _driver.write_us(_ch2, angleToUs(sagAngle));
        }

        template<class Driver>
        void ServoPWM<Driver>::disarm() {
            _driver.write_us(_ch1, SERVO_TRIM_US);
            _driver.write_us(_ch2, SERVO_TRIM_US);
        }

        template<class Driver>
        uint16_t ServoPWM<Driver>::angleToUs(float angle) {

            if (angle < ELEVON_MIN_ANGLE) angle = ELEVON_MIN_ANGLE;
            if (angle > ELEVON_MAX_ANGLE) angle = ELEVON_MAX_ANGLE;

            float norm = (angle - ELEVON_MIN_ANGLE) /
                        (ELEVON_MAX_ANGLE - ELEVON_MIN_ANGLE);

            norm = norm * 2.0f - 1.0f;

            // SERVO_MAX_US - SERVO_MIN_US * 0.5f
            float halfRange = 500.0f;

            return SERVO_TRIM_US + norm * halfRange;
        }

    } 
}