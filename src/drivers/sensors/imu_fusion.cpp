#include "imu_fusion.h"

#include <math.h>

using namespace atabey::drivers;

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
    ok &= mag.init();

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

    // 2) Angular rates straight from gyro (body frame, rad/s)
    p = imu.gx;
    q = imu.gy;
    r = imu.gz;

    // 3) Accel-derived angles (rad). NED, Z-down, at rest az ~ -g.
    float roll_acc  = atan2f(imu.ay, imu.az);
    float pitch_acc = atan2f(-imu.ax, sqrtf(imu.ay*imu.ay + imu.az*imu.az));

    // 4) Kalman predict + update for each axis
    phi   = kalmanUpdate(&kalmanRoll,  roll_acc,  p, dt);
    theta = kalmanUpdate(&kalmanPitch, pitch_acc, q, dt);

    // 5) Yaw from magnetometer (no fusion, raw heading).
    //    BMM150 may not produce fresh data every cycle; keep last psi if stale.
    if (mag.read()) {
        psi = atan2f(-mag.my, mag.mx);
    }

    // 6) Publish packed arrays
    sensorsAttitude[0] = phi;
    sensorsAttitude[1] = theta;
    sensorsAttitude[2] = psi;

    sensorsRates[0]    = p;
    sensorsRates[1]    = q;
    sensorsRates[2]    = r;
}
