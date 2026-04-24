#pragma once

#include "mpu6250.h"
#include "bmm150.h"

// ---------------------------------------------------------------------------
// Standardized autopilot state outputs — all in SI units.
// Updated exactly once per updateSensors() call at fixed timestep dt = 0.01s.
// ---------------------------------------------------------------------------

// Position (NED frame, metres). h is Down, positive downward.
extern float PN, PE, h;

// Velocity (Earth/NED frame, m/s).
extern float u, v, w;

// Angular rates (body frame, rad/s).
extern float p, q, r;

// Attitude (Euler angles, rad): roll, pitch, yaw.
extern float phi, theta, psi;

// Packed arrays for downstream consumers.
extern float sensorsPosition[3];    // { PN, PE, h }
extern float sensorsEarthspeed[3];  // { u, v, w }
extern float sensorsRates[3];       // { p, q, r }
extern float sensorsAttitude[3];    // { phi, theta, psi }

// Fixed integration timestep (seconds).
extern const float dt;

// Initialise sensors + fusion state. Call once before updateSensors().
bool initSensors();

// Main periodic task: read sensors, run fusion, publish outputs.
// Must be invoked at 1 / dt Hz (100 Hz) from a deterministic scheduler.
void updateSensors();
