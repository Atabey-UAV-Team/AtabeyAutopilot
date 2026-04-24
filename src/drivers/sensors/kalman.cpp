#include "kalman.h"

void kalmanInit(Kalman1D* k) {
    k->angle     = 0.0f;
    k->bias      = 0.0f;

    k->P[0][0]   = 0.0f;
    k->P[0][1]   = 0.0f;
    k->P[1][0]   = 0.0f;
    k->P[1][1]   = 0.0f;

    k->Q_angle   = 0.001f;
    k->Q_bias    = 0.003f;
    k->R_measure = 0.03f;
}

float kalmanUpdate(Kalman1D* k, float newAngle, float newRate, float dt) {
    // -----------------------------------------------------------------------
    // 1) Predict
    // -----------------------------------------------------------------------
    float rate = newRate - k->bias;
    k->angle  += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    // -----------------------------------------------------------------------
    // 2) Update — innovation, Kalman gain
    // -----------------------------------------------------------------------
    float S    = k->P[0][0] + k->R_measure;     // innovation covariance
    float K0   = k->P[0][0] / S;                // Kalman gain (angle)
    float K1   = k->P[1][0] / S;                // Kalman gain (bias)

    float y    = newAngle - k->angle;           // innovation
    k->angle  += K0 * y;
    k->bias   += K1 * y;

    // -----------------------------------------------------------------------
    // 3) Covariance update — use cached P00, P01 (must not overwrite mid-calc)
    // -----------------------------------------------------------------------
    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K0 * P00_temp;
    k->P[0][1] -= K0 * P01_temp;
    k->P[1][0] -= K1 * P00_temp;
    k->P[1][1] -= K1 * P01_temp;

    return k->angle;
}
