#pragma once

// 1D Kalman filter (2-state: angle, gyro bias) per axis.
// Reference: classical Lauszus form. Embedded-safe: POD struct, no heap, no STL.

typedef struct {
    float angle;        // estimated angle (rad)
    float bias;         // estimated gyro bias (rad/s)
    float P[2][2];      // error covariance matrix
    float Q_angle;      // process noise variance — angle
    float Q_bias;       // process noise variance — gyro bias
    float R_measure;    // measurement noise variance — accel-derived angle
} Kalman1D;

// Initialise filter to a known-zero state with the recommended tuning.
void  kalmanInit(Kalman1D* k);

// One step: predict with gyro rate, correct with accel-derived angle.
//   newAngle : measured angle from accelerometer (rad)
//   newRate  : raw gyro rate, body axis (rad/s)
//   dt       : timestep (s)
// Returns the updated angle estimate.
float kalmanUpdate(Kalman1D* k, float newAngle, float newRate, float dt);
