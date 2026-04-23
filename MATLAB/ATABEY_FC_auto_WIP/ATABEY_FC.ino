#include <math.h>

// Global değişkenler
const float dt              = 0.01f;    // Sample time
const float saturationAngle = 0.3491f;  // +- 20 derece, kontrol yüzeyi max hareket açısı

// Sensörden gelen veriler
float PN, PE, h;        // NED'de orijine göre konum: North, East, Ze
float u, v, w;          // Earth-frame hızlar: North, East, Down (m/s)
float p, q, r;          // Gövde açısal hızlar: roll, pitch, yaw (rad/s)
float phi, theta, psi;  // Euler açıları (rad)

// Sensör verisi dizileri
float sensorsPosition[3];    // { PN, PE, Ze }   Ze aşağı
float sensorsEarthspeed[3];  // { u,  v,  w  }
float sensorsRates[3];       // { p,  q,  r  }
float sensorsAttitude[3];    // { phi, theta, psi }

float vehicleGroundspeed;    // Aracın yere göre hız
float vehicleHeading;        // Yere göre açı (rad)

/*
// Otopilot pozisyon ve hız referans bilgisi
float fcPosRef[3];  // { posNorth, posEast, altitude_up } (m)
float fcVelRef[3];  // { velNorth, velEast,  velDown    } (m/s)
*/

// PID Katsayıları
  // FC1 — Trajectory tracker
  const float Kp_Xe = 0.2f;
  const float Kp_Ye = 0.5f,  Ki_Ye = 0.05f;

  // FC2 — Autopilot
  const float Kp_Ze = 0.8f,  Ki_Ze = 0.05f, Kd_Ze  = 0.4f;
  const float Kp_psi = 2.0f, Ki_psi = 0.1f;
  const float Kp_u  = 0.5f,  Ki_u  = 0.05f;

  // FC3 — Attitude controller
  const float Kp_theta = 2.0f, Ki_theta = 0.1f;
  const float Kp_phi   = 2.0f, Ki_phi   = 0.1f;

  // FC4 — SAS
  const float Kq_eta    = 0.3f;
  const float Ktheta_eta = 0.2f;
  const float Kphi_xi   = 0.2f;

// Her kontrol döngüsü için farklı PID struct yapısı
struct PIDState {
  float integral  = 0.0f;
  float lastError = 0.0f;
};

PIDState pid_Ye;     // FC1: lateral cross-track
PIDState pid_Ze;     // FC2: altitude
PIDState pid_psi;    // FC2: heading
PIDState pid_u;      // FC2: speed
PIDState pid_theta;  // FC3: pitch
PIDState pid_phi;    // FC3: roll

float computePID(PIDState &state,
                 float sampleTime,
                 float Kp, float Ki, float Kd,
                 float setpoint, float input,
                 float satUpper =  1e9f,
                 float satLower = -1e9f) {

  float error       = setpoint - input;
  state.integral   += error * sampleTime;
  float derivative  = (error - state.lastError) / sampleTime;
  state.lastError   = error;

  float output = Kp * error + Ki * state.integral + Kd * derivative;
  return constrain(output, satLower, satUpper);
}

// Loop başında sensör verisini günceller
void updateSensors() {
  // TODO: Sinyal girdileri yapılacak

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

/*
  Manuel uçuş için FC1 ve FC2 kullanılmıyor

// FC 1 Helper - "track errors in path frame" bloğu
// Y = { OTD, dOTD, ATD }
//   OTD  : cross-track distance  (m)
//   dOTD : cross-track speed     (m/s)
//   ATD  : along-track distance  (m)
void track(float Vel_UAV[3], float Pos_UAV[3],
           float Vel_VL[3],  float Pos_VL[3],
           float Y[3]) {

  float ChiVL = atan2f(Vel_VL[1], Vel_VL[0]);

  float d_NED[3] = {
    Pos_UAV[0] - Pos_VL[0],
    Pos_UAV[1] - Pos_VL[1],
    Pos_UAV[2] - Pos_VL[2]
  };

  float dV_NED[3] = {
    Vel_UAV[0] - Vel_VL[0],
    Vel_UAV[1] - Vel_VL[1],
    Vel_UAV[2] - Vel_VL[2]
  };

  float c = cosf(ChiVL);
  float s = sinf(ChiVL);

  Y[0] = -s * d_NED[0]  + c * d_NED[1];   // OTD
  Y[1] = -s * dV_NED[0] + c * dV_NED[1];  // dOTD
  Y[2] =  c * d_NED[0]  + s * d_NED[1];   // ATD
}

// FC 1 - Trajectory Tracker
// Output: trajectoryOut[3] = { h_d, psi_d, V_d }
void trajectoryTracker(float trajectoryOut[3]) {

  float h_d = fcPosRef[2];   // Referans irtifa, yukarı yön pozitif (m)

  float trackerOut[3];
  track(fcVelRef, fcPosRef, sensorsEarthspeed, sensorsPosition, trackerOut);

  float OTD = trackerOut[0];   // cross-track error
  float ATD = trackerOut[2];   // along-track error

  // Heading demand: path heading + PI cross-track correction
  // atan2(velEast, velNorth) gives the virtual-leader course
  float ChiVL = atan2f(fcVelRef[1], fcVelRef[0]);
  float lateralCorr = computePID(pid_Ye, dt, Kp_Ye, Ki_Ye, 0.0f,
                                  0.0f, OTD,            // error = 0 - OTD
                                  saturationAngle, -saturationAngle);
  float psi_d = ChiVL + lateralCorr;

  // Speed demand: close along-track gap; clamp to safe airspeed band
  float V_d = constrain(-ATD * Kp_Xe + vehicleGroundspeed, 18.0f, 90.0f);

  trajectoryOut[0] = h_d;
  trajectoryOut[1] = psi_d;
  trajectoryOut[2] = V_d;
}

// ================================================================
// FC 2 — Autopilot
// Input:  trajectoryOut[3] = { h_d,    psi_d, V_d   }
// Output: attitude_d[3]    = { theta_d, phi_d, tau_d }
// ================================================================
void autopilot(float trajectoryOut[3], float attitude_d[3]) {

  float h_d   = trajectoryOut[0];
  float psi_d = trajectoryOut[1];
  float V_d   = trajectoryOut[2];

  // --- Altitude Hold → theta_d ---
  // "Altitude to Ze" block: Ze_d = -h_d (NED sign flip).
  // Kd branch feeds Ze_rate directly (sensorsEarthspeed[2] = w = Zdot)
  // rather than the computed error derivative — matches Simulink layout.
  float theta_d = computePID(pid_Ze, dt, Kp_Ze, Ki_Ze, 0.0f,
                              -h_d, sensorsPosition[2])  // Ze error
                - Kd_Ze * sensorsEarthspeed[2];           // direct Ze_rate damping

  // --- Heading Hold → phi_d ---
  // Simulink "complex division = angle subtraction" block:
  //   angle( e^(j·psi_d) / e^(j·chi) ) = psi_d − chi, wrapped to [−π, π]
  float headingErr = atan2f(sinf(psi_d - vehicleHeading),
                             cosf(psi_d - vehicleHeading));

  // PI on wrapped heading error; clamped by "Saturation roll" block
  pid_psi.integral += headingErr * dt;
  float phi_d = constrain(Kp_psi * headingErr + Ki_psi * pid_psi.integral,
                          -saturationAngle, saturationAngle);

  // --- Velocity Hold → tau_d ---
  // Unit negative feedback PI; clamped by "Saturation Throttle" block
  float tau_d = computePID(pid_u, dt, Kp_u, Ki_u, 0.0f,
                            V_d, vehicleGroundspeed,
                            1.0f, 0.0f);    // throttle in [0, 1]

  attitude_d[0] = theta_d;
  attitude_d[1] = phi_d;
  attitude_d[2] = tau_d;
}
*/

// FC 3 - Attitude Controller
// Input:  attitude_d[3] = { theta_d, phi_d,  tau_d }
// Output: demands[3]    = { eta_d,   xi_d,   tau_d }
//   eta_d : elevator demand (rad)
//   xi_d  : aileron demand  (rad)
void attitudeController(float attitude_d[3], float demands[3]) {

  float theta_d = attitude_d[0];
  float phi_d   = attitude_d[1];
  float tau_d   = attitude_d[2];

  // Pitch: desired pitch angle → elevator pre-command
  float eta_d = computePID(pid_theta, dt, Kp_theta, Ki_theta, 0.0f,
                            theta_d, sensorsAttitude[1],     // theta feedback
                            saturationAngle, -saturationAngle);

  // Roll: desired roll angle → aileron pre-command
  float xi_d = computePID(pid_phi, dt, Kp_phi, Ki_phi, 0.0f,
                           phi_d, sensorsAttitude[0],        // phi feedback
                           saturationAngle, -saturationAngle);

  demands[0] = eta_d;
  demands[1] = xi_d;
  demands[2] = tau_d;
}

// FC 4 — Stability Augmentation System (SAS)
// Input:  demands[3]  = { eta_d, xi_d, tau_d }
// Output: controls[4] = { elevatorCmd, aileronCmd, rudderCmd, throttleCmd }
void SAS(float demands[3], float controls[4]) {

  float eta_d = demands[0];
  float xi_d  = demands[1];
  float tau_d = demands[2];

  // Elevator: pitch-rate damping + pitch-angle feedback
  float elevatorCmd = eta_d
                    - Kq_eta    * sensorsRates[1]       // q damping
                    - Ktheta_eta * sensorsAttitude[1];  // theta feedback

  // Aileron: roll-angle feedback
  float aileronCmd = xi_d - Kphi_xi * sensorsAttitude[0];

  controls[0] = elevatorCmd;
  controls[1] = aileronCmd;
  controls[2] = 0.0f;   // Rudder yok
  controls[3] = tau_d;  // Throttle'a işlem yapılmıyor
}

void setup() {
  Serial.begin(115200);
  // TODO: Sensörler başlatılacak
}

void loop() {
  // Sensör oku
  updateSensors();

  /*
  // FC1 çalıştır
  float trajectoryOut[3];
  trajectoryTracker(trajectoryOut);

  // FC2 çalıştır
  float attitude_d[3];
  autopilot(trajectoryOut, attitude_d);
  */

  // FC 3 çalıştır
  float demands[3];
  attitudeController(attitude_d, demands);

  // FC 4 çalıştır
  float controls[4];
  SAS(demands, controls);

  // TODO: "controls" verileri motorlara gönderilecek

  delay((int)(dt * 1000));  // PID'de float olarak kullanılan dt burda delay() fonksiyonu için integer'a dönüştürüldü
}
```