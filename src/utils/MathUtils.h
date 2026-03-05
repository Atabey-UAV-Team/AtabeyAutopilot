#pragma once
#include <math.h>

namespace atabey {
    namespace utils {
        
        struct Vec3f {
            float x{0.0f};
            float y{0.0f};
            float z{0.0f};

            Vec3f() = default;
            Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

            Vec3f operator+(const Vec3f& other) const {
                return {x + other.x, y + other.y, z + other.z};
            }

            Vec3f operator-(const Vec3f& other) const {
                return {x - other.x, y - other.y, z - other.z};
            }

            Vec3f operator*(float scalar) const {
                return {x * scalar, y * scalar, z * scalar};
            }
        };

        inline float derivative(float prev, float current, float dt) {
            return (current - prev) / dt;
        }

        inline float clamp(float v, float minVal, float maxVal) {
            if (v < minVal) return minVal;
            if (v > maxVal) return maxVal;
            return v;
        }

        inline float deg2rad(float deg) {
            return deg * 0.01745329251f;
        }

        inline float rad2deg(float rad) {
            return rad * 57.295779513f;
        }

        inline float wrapPi(float rad) {
            while (rad > PI)  rad -= TWO_PI;
            while (rad < -PI) rad += TWO_PI;
            return rad;
        }

        inline float lerp(float a, float b, float t) {
            return a + (b - a) * t;   // t: 0..1
        }

        inline Vec3f lerp(const Vec3f& a, const Vec3f& b, float t) {
            return {
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t
            };
}

        inline float applyDeadzone(float v, float dz) {
            if (fabsf(v) < dz) return 0.0f;

            if (v > 0.0f)
                return (v - dz) / (1.0f - dz);
            else
                return (v + dz) / (1.0f - dz);
        }

        inline float angleError(float target, float current) {
            return wrapPi(target - current);
        }

    }
}