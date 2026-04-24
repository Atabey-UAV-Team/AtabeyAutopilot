#include "imu_fusion.h"

#include <math.h>

using namespace atabey::drivers;

// ---------------------------------------------------------------------------
// Global state — all allocated statically. No heap.
// ---------------------------------------------------------------------------

const float dt = 0.01f;   // 100 Hz fixed timestep

float PN = 0.0f, PE = 0.0f, h = 0.0f;
float u  = 0.0f, v  = 0.0f, w = 0.0f;
float p  = 0.0f, q  = 0.0f, r = 0.0f;
float phi = 0.0f, theta = 0.0f, psi = 0.0f;

float sensorsPosition[3]   = {0.0f, 0.0f, 0.0f};
float sensorsEarthspeed[3] = {0.0f, 0.0f, 0.0f};
float sensorsRates[3]      = {0.0f, 0.0f, 0.0f};
float sensorsAttitude[3]   = {0.0f, 0.0f, 0.0f};

// ---------------------------------------------------------------------------
// File-local state. All static — deterministic and heap-free.
// ---------------------------------------------------------------------------

namespace {

    MPU6250 imu;
    BMM150  mag;

    // Mahony attitude quaternion (NED, body X-fwd, Y-right, Z-down).
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    // Mahony gains and integral feedback (gyro bias estimate).
    constexpr float MAHONY_KP = 2.0f;   // proportional gain on accel/mag error
    constexpr float MAHONY_KI = 0.005f; // integral gain -> drives out gyro bias
    float ix = 0.0f, iy = 0.0f, iz = 0.0f;

    // Constants
    constexpr float GRAVITY = 9.80665f;

    // Velocity / position damping to suppress integrator drift without GPS aiding.
    // Basic first-order leak: v *= (1 - k*dt). Keep conservative.
    constexpr float VEL_DAMPING = 0.20f; // 1/s
    constexpr float POS_DAMPING = 0.02f; // 1/s

    // Accelerometer zero-velocity detection threshold (m/s^2, residual after gravity removal).
    constexpr float ZUPT_ACC_EPS = 0.25f;

    inline void normalize3(float& x, float& y, float& z) {
        float n = sqrtf(x*x + y*y + z*z);
        if (n > 1.0e-6f) {
            float inv = 1.0f / n;
            x *= inv; y *= inv; z *= inv;
        }
    }

    // ------------------------------------------------------------------------
    // Mahony AHRS update (x-io reference form, adapted for NED Z-down body).
    //
    // gx,gy,gz : body rates (rad/s)
    // ax,ay,az : specific-force from accel (m/s^2). At rest in NED Z-down body,
    //            az ~ -g. We pass raw sensor values; internally we negate to
    //            obtain the gravity *direction* vector in body frame.
    // mx,my,mz : magnetometer (uT), body frame.
    // ------------------------------------------------------------------------
    void mahonyUpdate(float gx_, float gy_, float gz_,
                      float ax_, float ay_, float az_,
                      float mx_, float my_, float mz_,
                      float ts)
    {
        // Use mag only if valid (non-zero)
        const bool useMag = !(mx_ == 0.0f && my_ == 0.0f && mz_ == 0.0f);

        // Accel must be valid too, else skip feedback (propagate gyro only)
        const float a_norm_sq = ax_*ax_ + ay_*ay_ + az_*az_;
        const bool useAcc = (a_norm_sq > 1.0e-4f);

        if (useAcc) {
            // Gravity direction in body frame = -specific_force / |specific_force|
            float ax = -ax_, ay = -ay_, az = -az_;
            normalize3(ax, ay, az);

            // Estimated half-gravity direction in body (from quaternion).
            // Reference gravity in NED = (0, 0, 1).
            float halfvx = q1*q3 - q0*q2;
            float halfvy = q0*q1 + q2*q3;
            float halfvz = q0*q0 - 0.5f + q3*q3;

            // Error = measured x estimated (body frame)
            float halfex = (ay * halfvz - az * halfvy);
            float halfey = (az * halfvx - ax * halfvz);
            float halfez = (ax * halfvy - ay * halfvx);

            if (useMag) {
                float mx = mx_, my = my_, mz = mz_;
                normalize3(mx, my, mz);

                // Rotate measured mag into earth frame
                float hx = 2.0f * (mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3)       + mz*(q1*q3 + q0*q2));
                float hy = 2.0f * (mx*(q1*q2 + q0*q3)       + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
                float hz = 2.0f * (mx*(q1*q3 - q0*q2)       + my*(q2*q3 + q0*q1)        + mz*(0.5f - q1*q1 - q2*q2));

                // Reference magnetic field: horizontal (bx) and vertical (bz) only.
                float bx = sqrtf(hx*hx + hy*hy);
                float bz = hz;

                // Estimated direction of mag in body frame
                float halfwx = bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2);
                float halfwy = bx*(q1*q2 - q0*q3)        + bz*(q0*q1 + q2*q3);
                float halfwz = bx*(q0*q2 + q1*q3)        + bz*(0.5f - q1*q1 - q2*q2);

                halfex += (my * halfwz - mz * halfwy);
                halfey += (mz * halfwx - mx * halfwz);
                halfez += (mx * halfwy - my * halfwx);
            }

            // Integral feedback
            if (MAHONY_KI > 0.0f) {
                ix += MAHONY_KI * halfex * ts;
                iy += MAHONY_KI * halfey * ts;
                iz += MAHONY_KI * halfez * ts;
                gx_ += ix;
                gy_ += iy;
                gz_ += iz;
            }

            // Proportional feedback
            gx_ += MAHONY_KP * halfex;
            gy_ += MAHONY_KP * halfey;
            gz_ += MAHONY_KP * halfez;
        }

        // Integrate quaternion using body rates (first-order, fixed dt)
        float half_dt = 0.5f * ts;
        gx_ *= half_dt;
        gy_ *= half_dt;
        gz_ *= half_dt;

        float qa = q0, qb = q1, qc = q2;
        q0 += (-qb * gx_ - qc * gy_ - q3 * gz_);
        q1 += ( qa * gx_ + qc * gz_ - q3 * gy_);
        q2 += ( qa * gy_ - qb * gz_ + q3 * gx_);
        q3 += ( qa * gz_ + qb * gy_ - qc * gx_);

        // Renormalise quaternion
        float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (qn > 1.0e-9f) {
            float inv = 1.0f / qn;
            q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv;
        } else {
            q0 = 1.0f; q1 = q2 = q3 = 0.0f;
        }
    }

    // Rotate a body-frame vector into NED using current quaternion.
    inline void bodyToNED(float bx, float by, float bz,
                          float& nx, float& ny, float& nz)
    {
        float r11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
        float r12 = 2.0f * (q1*q2 - q0*q3);
        float r13 = 2.0f * (q1*q3 + q0*q2);
        float r21 = 2.0f * (q1*q2 + q0*q3);
        float r22 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        float r23 = 2.0f * (q2*q3 - q0*q1);
        float r31 = 2.0f * (q1*q3 - q0*q2);
        float r32 = 2.0f * (q2*q3 + q0*q1);
        float r33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        nx = r11*bx + r12*by + r13*bz;
        ny = r21*bx + r22*by + r23*bz;
        nz = r31*bx + r32*by + r33*bz;
    }

    inline void quatToEuler(float& roll, float& pitch, float& yaw) {
        // ZYX (yaw-pitch-roll) aerospace convention
        float sinp = 2.0f * (q0*q2 - q3*q1);
        if (sinp >  1.0f) sinp =  1.0f;
        if (sinp < -1.0f) sinp = -1.0f;

        roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
        pitch = asinf(sinp);
        yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
    }

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool initSensors() {
    bool ok = true;
    ok &= imu.init();
    ok &= mag.init();

    // Quiescent gyro bias calibration — vehicle must be stationary at power-up.
    if (ok) {
        imu.calibrateGyro(500);
    }

    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    ix = iy = iz = 0.0f;

    PN = PE = h  = 0.0f;
    u  = v  = w  = 0.0f;
    p  = q  = r  = 0.0f;
    phi = theta = psi = 0.0f;

    return ok;
}

void updateSensors() {
    // 1) Sensor acquisition
    imu.read();
    mag.read();

    // 2) Angular rates straight from gyro (body frame, rad/s)
    p = imu.gx;
    q = imu.gy;
    r = imu.gz;

    // 3) Attitude fusion
    mahonyUpdate(p, q, r,
                 imu.ax, imu.ay, imu.az,
                 mag.mx, mag.my, mag.mz,
                 dt);

    quatToEuler(phi, theta, psi);

    // 4) Velocity in NED from gravity-compensated accel
    //    Body specific-force -> NED, then subtract gravity.
    //    In NED Z-down, g = (0, 0, +g). Specific force f_ned = a_ned - g_ned,
    //    so a_ned = f_ned + g_ned.
    float axN, ayN, azN;
    bodyToNED(imu.ax, imu.ay, imu.az, axN, ayN, azN);
    azN += GRAVITY;

    // Zero-velocity update: if residual accel is near zero, damp velocity hard.
    float accelMag = sqrtf(axN*axN + ayN*ayN + azN*azN);
    bool stationary = (accelMag < ZUPT_ACC_EPS);

    u += axN * dt;
    v += ayN * dt;
    w += azN * dt;

    // Damping (drift suppression)
    float velLeak = 1.0f - VEL_DAMPING * dt;
    if (stationary) velLeak = 1.0f - (VEL_DAMPING * 10.0f) * dt;
    if (velLeak < 0.0f) velLeak = 0.0f;
    u *= velLeak;
    v *= velLeak;
    w *= velLeak;

    // 5) Position integration (NED). h is Down component, positive downward.
    PN += u * dt;
    PE += v * dt;
    h  += w * dt;

    float posLeak = 1.0f - POS_DAMPING * dt;
    if (posLeak < 0.0f) posLeak = 0.0f;
    PN *= posLeak;
    PE *= posLeak;
    h  *= posLeak;

    // 6) Publish packed arrays
    sensorsAttitude[0]   = phi;
    sensorsAttitude[1]   = theta;
    sensorsAttitude[2]   = psi;

    sensorsRates[0]      = p;
    sensorsRates[1]      = q;
    sensorsRates[2]      = r;

    sensorsEarthspeed[0] = u;
    sensorsEarthspeed[1] = v;
    sensorsEarthspeed[2] = w;

    sensorsPosition[0]   = PN;
    sensorsPosition[1]   = PE;
    sensorsPosition[2]   = h;
}
