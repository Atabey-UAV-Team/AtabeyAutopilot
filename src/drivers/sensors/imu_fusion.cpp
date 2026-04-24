#include "imu_fusion.h"

#include <math.h>

using namespace atabey::drivers;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---------------------------------------------------------------------------
// Magnetometer hard-iron bias (uT). Populate from ground calibration.
// ---------------------------------------------------------------------------
static float mag_bias_x = 0.0f;
static float mag_bias_y = 0.0f;
static float mag_bias_z = 0.0f;

// Complementary-filter weight: trust gyro short-term, mag long-term.
static const float YAW_ALPHA = 0.98f;

// Expected Earth-field magnitude window (uT). Outside -> disturbance; reject.
static const float MAG_NORM_MIN = 20.0f;
static const float MAG_NORM_MAX = 70.0f;

static inline float wrapAngle(float x) {
    while (x >  M_PI) x -= 2.0f * M_PI;
    while (x < -M_PI) x += 2.0f * M_PI;
    return x;
}

// ---------------------------------------------------------------------------
// Global state — all allocated statically. No heap.
// ---------------------------------------------------------------------------

const float dt = 0.01f;   // 100 Hz fixed timestep

float phi = 0.0f, theta = 0.0f, psi = 0.0f;
float p   = 0.0f, q     = 0.0f, r   = 0.0f;

float sensorsAttitude[3] = {0.0f, 0.0f, 0.0f};
float sensorsRates[3]    = {0.0f, 0.0f, 0.0f};

Kalman1D kalmanRoll;
Kalman1D kalmanPitch;

// ---------------------------------------------------------------------------
// File-local sensor instances. Static — deterministic, heap-free.
// ---------------------------------------------------------------------------

namespace {
    MPU6250 imu;
    BMM150  mag;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool initSensors() {
    bool ok = true;
    ok &= imu.init();
    if (!ok) {
        Serial.println(F("MPU6250 init FAILED"));
    }
    ok &= mag.init();
    if (!ok) {
        Serial.println(F("BMM150 init FAILED"));
    }

    if (ok) {
        // Vehicle must be stationary at power-up for valid bias.
        imu.calibrateGyro(500);
    }

    kalmanInit(&kalmanRoll);
    kalmanInit(&kalmanPitch);

    phi = theta = psi = 0.0f;
    p   = q     = r   = 0.0f;

    return ok;
}

void updateSensors() {
    // 1) Read MPU6250
    imu.read();

    // 2) Accel-derived angles (rad). NED, Z-down, at rest az ~ -g.
    float roll_acc  = atan2f(imu.ay, imu.az);
    float pitch_acc = atan2f(-imu.ax, sqrtf(imu.ay*imu.ay + imu.az*imu.az));

    // 2a) Kalman predict + update for roll/pitch (unchanged)
    phi   = kalmanUpdate(&kalmanRoll,  roll_acc,  imu.gx, dt);
    theta = kalmanUpdate(&kalmanPitch, pitch_acc, imu.gy, dt);

    // 3) Body angular rates (rad/s)
    p = imu.gx;
    q = imu.gy;
    r = imu.gz;

    // 4) Gyro integration for yaw (dead-reckoning baseline).
    psi += r * dt;

    // 5) Read BMM150. If no fresh sample, skip mag correction entirely.
    if (mag.read()) {
        // 5a) Hard-iron bias removal.
        float mx_c = mag.mx - mag_bias_x;
        float my_c = mag.my - mag_bias_y;
        float mz_c = mag.mz - mag_bias_z;

        // 5b) Validity gating — reject disturbed / saturated readings.
        float mag_norm = sqrtf(mx_c*mx_c + my_c*my_c + mz_c*mz_c);

        if (mag_norm > MAG_NORM_MIN && mag_norm < MAG_NORM_MAX) {
            // 6) Tilt-compensated yaw (NED frame) using current phi, theta.
            float sp = sinf(phi);
            float cp = cosf(phi);
            float st = sinf(theta);
            float ct = cosf(theta);

            float mx2 = mx_c * ct + mz_c * st;
            float my2 = mx_c * sp * st
                      + my_c * cp
                      - mz_c * sp * ct;

            float psi_mag = atan2f(-my2, mx2);

            // 7) Complementary fusion. Unwrap delta to avoid ±π discontinuity.
            float delta = wrapAngle(psi_mag - psi);
            psi = psi + (1.0f - YAW_ALPHA) * delta;
        }
        // else: magnetic disturbance → gyro-only this cycle.
    }

    // 8) Keep yaw in [-pi, pi].
    psi = wrapAngle(psi);

    // 9) Publish packed arrays
    sensorsAttitude[0] = phi;
    sensorsAttitude[1] = theta;
    sensorsAttitude[2] = psi;

    sensorsRates[0]    = p;
    sensorsRates[1]    = q;
    sensorsRates[2]    = r;
}
