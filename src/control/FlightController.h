#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>

namespace atabey {
namespace control {

class FlightController {
public:
    FlightController();

    void updateSensors(float PN, float PE, float h,
                       float u, float v, float w,
                       float p, float q, float r,
                       float phi, float theta, float psi);

    void attitudeController(float attitude_d[3], float demands[3]);
    void SAS(float demands[3], float controls[4]);

private:
    static constexpr float dt = 0.01f;
    static constexpr float saturationAngle = 0.3491f;

    float PN, PE, h;
    float u, v, w;
    float p, q, r;
    float phi, theta, psi;

    float sensorsPosition[3];
    float sensorsEarthspeed[3];
    float sensorsRates[3];
    float sensorsAttitude[3];

    float vehicleGroundspeed;
    float vehicleHeading;

    static constexpr float Kp_theta = 2.0f, Ki_theta = 0.1f;
    static constexpr float Kp_phi   = 2.0f, Ki_phi   = 0.1f;

    static constexpr float Kq_eta    = 0.3f;
    static constexpr float Ktheta_eta = 0.2f;
    static constexpr float Kphi_xi   = 0.2f;

    struct PIDState {
        float integral;
        float lastError;
    };

    PIDState pid_theta;
    PIDState pid_phi;

    float computePID(PIDState &state,
                     float sampleTime,
                     float Kp, float Ki, float Kd,
                     float setpoint, float input,
                     float satUpper,
                     float satLower);
};

} // namespace control
} // namespace atabey

#endif