#include "FlightController.h"
#include <math.h>

namespace atabey {
namespace control {

// Global değişkenler
// const float dt              = 0.01f;    // Sample time
// const float saturationAngle = 0.3491f;  // +- 20 derece, kontrol yüzeyi max hareket açısı

// Sensörden gelen veriler
// float PN, PE, h;        // NED'de orijine göre konum: North, East, Ze
// float u, v, w;          // Earth-frame hızlar: North, East, Down (m/s)
// float p, q, r;          // Gövde açısal hızlar: roll, pitch, yaw (rad/s)
// float phi, theta, psi;  // Euler açıları (rad)

// Sensör verisi dizileri
// float sensorsPosition[3];    // { PN, PE, Ze }   Ze aşağı
// float sensorsEarthspeed[3];  // { u,  v,  w  }
// float sensorsRates[3];       // { p,  q,  r  }
// float sensorsAttitude[3];    // { phi, theta, psi }

// float vehicleGroundspeed;    // Aracın yere göre hız
// float vehicleHeading;        // Yere göre açı (rad)

// PID Katsayıları
// FC3 - Attitude controller
// const float Kp_theta = 2.0f, Ki_theta = 0.1f;
// const float Kp_phi   = 2.0f, Ki_phi   = 0.1f;

// FC4 - SAS
// const float Kq_eta    = 0.3f;
// const float Ktheta_eta = 0.2f;
// const float Kphi_xi   = 0.2f;

// Her kontrol döngüsü için farklı PID struct yapısı

FlightController::FlightController() {
    pid_theta = {0.0f, 0.0f};
    pid_phi   = {0.0f, 0.0f};
}

// Loop başında sensör verisini günceller
void FlightController::updateSensors(float PN_, float PE_, float h_,
                                     float u_, float v_, float w_,
                                     float p_, float q_, float r_,
                                     float phi_, float theta_, float psi_) {

    PN = PN_;
    PE = PE_;
    h  = h_;

    u = u_;
    v = v_;
    w = w_;

    p = p_;
    q = q_;
    r = r_;

    phi   = phi_;
    theta = theta_;
    psi   = psi_;

    sensorsPosition[0]  = PN;
    sensorsPosition[1]  = PE;
    sensorsPosition[2]  = h;     // NED, Z aşağı

    sensorsEarthspeed[0] = u;
    sensorsEarthspeed[1] = v;
    sensorsEarthspeed[2] = w;

    sensorsRates[0] = p;
    sensorsRates[1] = q;
    sensorsRates[2] = r;

    sensorsAttitude[0] = phi;
    sensorsAttitude[1] = theta;
    sensorsAttitude[2] = psi;

    // 
    vehicleGroundspeed = sqrtf(u*u + v*v + w*w);   // Aracın yere göre hızı
    vehicleHeading     = atan2f(v, u);             // Yere göre açı
}

float FlightController::computePID(PIDState &state,
                                   float sampleTime,
                                   float Kp, float Ki, float Kd,
                                   float setpoint, float input,
                                   float satUpper,
                                   float satLower) {

    float error       = setpoint - input;
    state.integral   += error * sampleTime;
    float derivative  = (error - state.lastError) / sampleTime;
    state.lastError   = error;

    float output = Kp * error + Ki * state.integral + Kd * derivative;

    if (output > satUpper) output = satUpper;
    if (output < satLower) output = satLower;

    return output;
}

// FC 3 - Attitude Controller
// Input:  attitude_d[3] = { theta_d, phi_d,  tau_d }
// Output: demands[3]    = { eta_d,   xi_d,   tau_d }
//   eta_d : elevator demand (rad)
//   xi_d  : aileron demand  (rad)
void FlightController::attitudeController(float attitude_d[3], float demands[3]) {

    float theta_d = attitude_d[0];
    float phi_d   = attitude_d[1];
    float tau_d   = attitude_d[2];

    // Pitch: desired pitch angle → elevator pre-command
    float eta_d = computePID(pid_theta, dt, Kp_theta, Ki_theta, 0.0f,
                             theta_d, sensorsAttitude[1],
                             saturationAngle, -saturationAngle);

    // Roll: desired roll angle → aileron pre-command
    float xi_d = computePID(pid_phi, dt, Kp_phi, Ki_phi, 0.0f,
                            phi_d, sensorsAttitude[0],
                            saturationAngle, -saturationAngle);

    demands[0] = eta_d;
    demands[1] = xi_d;
    demands[2] = tau_d;
}

// FC 4 - Stability Augmentation System (SAS)
// Input:  demands[3]  = { eta_d, xi_d, tau_d }
// Output: controls[4] = { elevatorCmd, aileronCmd, rudderCmd, throttleCmd }
void FlightController::SAS(float demands[3], float controls[4]) {

    float eta_d = demands[0];
    float xi_d  = demands[1];
    float tau_d = demands[2];

    // Elevator: pitch-rate damping + pitch-angle feedback
    float elevatorCmd = eta_d
                      - Kq_eta    * sensorsRates[1]
                      - Ktheta_eta * sensorsAttitude[1];

    // Aileron: roll-angle feedback
    float aileronCmd = xi_d - Kphi_xi * sensorsAttitude[0];

    controls[0] = elevatorCmd;
    controls[1] = aileronCmd;
    controls[2] = 0.0f;   // Rudder yok
    controls[3] = tau_d;  // Throttle'a işlem yapılmıyor
}

} // namespace control
} // namespace atabey