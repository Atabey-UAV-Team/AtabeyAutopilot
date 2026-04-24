#pragma once

#include "mpu6250.h"
#include "bmm150.h"
#include "kalman.h"

// ---------------------------------------------------------------------------
// Standardized autopilot state outputs.
// Updated exactly once per updateSensors() call at fixed timestep dt = 0.01 s.
// ---------------------------------------------------------------------------

// Attitude (Euler angles, rad): roll, pitch, yaw.
extern float phi, theta, psi;

// Angular rates (body frame, rad/s).
extern float p, q, r;

// Packed arrays for downstream consumers.
extern float sensorsAttitude[3];   // { phi, theta, psi }
extern float sensorsRates[3];      // { p, q, r }

// Fixed integration timestep (seconds).
extern const float dt;

// Kalman filter instances (exposed for inspection / parameter tuning).
extern Kalman1D kalmanRoll;
extern Kalman1D kalmanPitch;

// Initialise sensors + filter state. Call once before updateSensors().
bool initSensors();

// Main periodic task: read sensors, run Kalman, publish outputs.
// Must be invoked at 1 / dt Hz (100 Hz) from a deterministic scheduler.
void updateSensors();
