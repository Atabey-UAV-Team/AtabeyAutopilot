// =============================================================
// PIN WIRING TABLE
// =============================================================
// Function          MCU Pin         Hardware
// -------------------------------------------------------------
// LEFT ELEVON       D6  (OC4A)      Servo Left Elevon
// RIGHT ELEVON      D7  (OC4B)      Servo Right Elevon
// THROTTLE (ESC)    D8  (OC4C)      ESC Signal
// ROLL INPUT        D2              Roll Signal (RC)
// PITCH INPUT       D3              Pitch Signal (RC)
// THROTTLE INPUT    D18             Throttle Signal (RC)
// -------------------------------------------------------------
// IMU MPU-6250      SDA=D20 SCL=D21 I2C @ 0x68 (Mega)
// MAG BMM150        SDA=D20 SCL=D21 I2C (shared bus)
// GPS (DISABLED)    RX1=D19 TX1=D18 Serial1 -- CONFLICTS with
//                                   RC throttle (D18); not used
//                                   in this build. See USE_GPS.
// =============================================================

#include <AtabeyAutopilot.h>
#include <drivers/sensors/imu_fusion.h>  // phi,theta,psi,p,q,r + initSensors()/updateSensors()
#include <math.h>   // fabsf() for disarm-gesture pitch check

// =============================================================
// ucus_test.ino
// MANUAL FLIGHT TEST (Flying Wing / Elevon Mix) - SAFETY FOCUSED
//
// Modules used:
//   - atabey::comm::Receiver
//   - atabey::drivers::ServoPWM
//   - atabey::drivers::Timer4ServoDriver
//   - drivers/sensors/imu_fusion  (MPU6250 + BMM150 + Kalman +
//                                  complementary-filter yaw)
//
// NO PID beyond FlightController, NO dynamic allocation,
// NO blocking delays in the main loop.
// =============================================================

using namespace atabey::comm;
using namespace atabey::drivers;
using namespace atabey::control;

// -------------------------------------------------------------
// Attitude stabilization limits
// -------------------------------------------------------------
#define MAX_ANGLE        0.35f
#define SAT_ANGLE        0.3491f

// -------------------------------------------------------------
// Direction configuration
// -------------------------------------------------------------
#define ROLL_DIR   1.0f
#define PITCH_DIR  1.0f

// -------------------------------------------------------------
// Timer4 channel mapping
// -------------------------------------------------------------
#define THROTTLE_OUT_CH 2

// -------------------------------------------------------------
// Safety timings
// -------------------------------------------------------------
#define SAFE_STARTUP_HOLD_MS    3000UL
#define ARM_STABLE_MS           2000UL
#define FAILSAFE_TIMEOUT_US   100000UL

// -------------------------------------------------------------
// RC raw-pulse plausibility window (us)
// -------------------------------------------------------------
#define RX_RAW_MIN_US  900
#define RX_RAW_MAX_US  2100

#define ARM_THROTTLE_MAX 0.05f

// -------------------------------------------------------------
// Scheduling periods (us). Independent non-blocking schedulers.
// Control loop 150 Hz; sensor fusion matches imu_fusion dt=0.01s (100 Hz).
// -------------------------------------------------------------
#define LOOP_PERIOD_US      6666UL    // ~150 Hz control
#define SENSOR_PERIOD_US   10000UL    // 100 Hz IMU fusion (dt=0.01s)

#define DEBUG_PERIOD_MS     100UL     // 10 Hz debug print

// GPS currently disabled: Serial1 TX1 (D18) collides with the RC
// throttle input and gps.init() uses delay(). Re-enable only after
// the wiring conflict is resolved.
#define USE_GPS 0

// -------------------------------------------------------------
// Instances
// -------------------------------------------------------------
Receiver           receiver;
Timer4ServoDriver  pwmDriver;
ServoPWM<Timer4ServoDriver> elevons(pwmDriver, 0, 1);
FlightController   fc;

// -------------------------------------------------------------
// State
// -------------------------------------------------------------
static bool     armed        = false;
static bool     failsafe     = true;
static bool     rearmLockout = false;
static bool     sensorsOk    = false;

static uint32_t bootMs            = 0;
static uint32_t lastRxOkUs        = 0;
static uint32_t throttleLowSince  = 0;
static uint32_t lastLoopUs        = 0;
static uint32_t lastSensorUs      = 0;
static uint32_t lastDbgMs         = 0;

static uint32_t disarmStickSince  = 0;
#define DISARM_STICK_HOLD_MS   2000UL
#define DISARM_ROLL_THRESHOLD  -0.9f

// -------------------------------------------------------------
// Loop timing / jitter tracking
// -------------------------------------------------------------
static uint32_t loopExecUs   = 0;
static uint32_t loopExecMin  = 0xFFFFFFFFUL;
static uint32_t loopExecMax  = 0;
static uint32_t loopExecAcc  = 0;
static uint32_t loopExecCnt  = 0;

// -------------------------------------------------------------
// Loop Working Variables (pre-allocated, static — no stack churn)
// -------------------------------------------------------------
static uint32_t nowUs          = 0;
static uint32_t nowMs          = 0;
static uint32_t loopStartUs    = 0;

static int16_t  rollPct        = 0;
static int16_t  pitchPct       = 0;
static int16_t  throttlePct    = 0;
static uint16_t rollUs         = 0;
static uint16_t pitchUs        = 0;
static uint16_t throttleUs     = 0;

static bool     rawValid       = false;
static bool     timedOut       = false;

static float    roll           = 0.0f;
static float    pitch          = 0.0f;
static float    throttle       = 0.0f;

static bool     inStartupHold  = false;

static float    left           = 0.0f;
static float    right          = 0.0f;
static float    leftDeg        = 0.0f;
static float    rightDeg       = 0.0f;

static uint16_t thrUs          = 1000;

// Stabilization scratch
static float    theta_d        = 0.0f;
static float    phi_d          = 0.0f;
static float    tau_d          = 0.0f;
static float    attitude_d[3]  = {0.0f, 0.0f, 0.0f};
static float    demands[3]     = {0.0f, 0.0f, 0.0f};
static float    controls[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
static float    pitchCmd       = 0.0f;
static float    rollCmd        = 0.0f;

// Sensor snapshot fed to FlightController (consistent within a cycle).
static float    s_phi   = 0.0f, s_theta = 0.0f, s_psi = 0.0f;
static float    s_p     = 0.0f, s_q     = 0.0f, s_r   = 0.0f;

// -------------------------------------------------------------
// Helpers (inline, heap-free)
// -------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline bool rawLooksValid(uint16_t us) {
    return (us >= RX_RAW_MIN_US && us <= RX_RAW_MAX_US);
}

static inline float normBipolar(int16_t pct) {
    return clampf((float)pct / 100.0f, -1.0f, 1.0f);
}

static inline float normUnipolar(int16_t pct) {
    return clampf((float)pct / 100.0f, 0.0f, 1.0f);
}

static inline float normToElevonDeg(float x) {
    return clampf(x, -1.0f, 1.0f) * 20.0f;
}

static inline uint16_t throttleToUs(float t) {
    t = clampf(t, 0.0f, 1.0f);
    return (uint16_t)(1000.0f + t * 1000.0f);
}

static inline void applySafeOutputs() {
    elevons.setPosition(0.0f, 0.0f);
    pwmDriver.write_us(THROTTLE_OUT_CH, 1000);
}

// -------------------------------------------------------------
// Sensor task: fixed-rate (100 Hz). Reads IMU + MAG, runs Kalman
// + tilt-compensated yaw complementary filter (alpha=0.98, inside
// the driver). Publishes globals phi/theta/psi, p/q/r.
// -------------------------------------------------------------
static inline void sensorTask() {
    if (!sensorsOk) return;
    updateSensors();   // from imu_fusion.h
}

// -------------------------------------------------------------
// Control task: reads latest sensor snapshot, feeds FlightController.
// -------------------------------------------------------------
static inline void feedFlightController() {
    // Snapshot — updateSensors() runs in main context (no ISR race),
    // but keep explicit copies so subsequent FC stages see coherent data.
    s_phi   = phi;
    s_theta = theta;
    s_psi   = psi;
    s_p     = p;
    s_q     = q;
    s_r     = r;

    // PN, PE, h, u, v, w: 0 until GPS / airspeed fusion are wired up.
    fc.updateSensors(
        0.0f, 0.0f, 0.0f,        // PN, PE, h
        0.0f, 0.0f, 0.0f,        // u,  v,  w
        s_p,  s_q,  s_r,         // p,  q,  r
        s_phi, s_theta, s_psi);  // phi, theta, psi
}

// =============================================================
// setup()
// =============================================================
void setup() {
    Serial.begin(115200);

    Serial.println(F("=== ucus_test :: SAFE MANUAL FLYING-WING TEST ==="));
    Serial.println(F("Booting DISARMED. Throttle forced to 1000us."));
    Serial.println(F("Startup hold: 3s. Arm requires throttle<5% for 2s."));

    receiver.init();
    elevons.init();

    // Sensors: initSensors() brings up Wire, MPU6250, BMM150 and
    // calibrates gyro bias (vehicle MUST be stationary at boot).
    sensorsOk = initSensors();
    if (!sensorsOk) {
        Serial.println(F("SENSOR INIT FAILED -- flying blind; FC fed zeros."));
    }

    applySafeOutputs();

    // IMU + mag init. Vehicle must be stationary for gyro bias calibration.
    // Failure does NOT block boot, but imuReady stays false → arming blocked.
    imuInitOk = initSensors();
    if (!imuInitOk) {
        Serial.println(F("IMU init FAILED - aircraft will remain disarmed."));
    }

    bootMs           = millis();
    lastRxOkUs       = micros();
    lastLoopUs       = micros();
    lastSensorUs     = micros();
    lastDbgMs        = millis();
    throttleLowSince = 0;

    armed    = false;
    failsafe = true;
}

// =============================================================
// loop() - non-blocking; runs sensor @100Hz and control @150Hz
// on independent schedulers. No delay(), no dynamic allocation.
// =============================================================
void loop() {
    loopStartUs = micros();

    // -----------------------------------------------------------
    // Sensor fusion schedule (100 Hz / 10 ms)
    //   Fixed-step catch-up: advance lastSensorUs by one period so
    //   long-term rate stays locked to SENSOR_PERIOD_US regardless
    //   of jitter in this tick.
    // -----------------------------------------------------------
    if ((uint32_t)(loopStartUs - lastSensorUs) >= SENSOR_PERIOD_US) {
        lastSensorUs += SENSOR_PERIOD_US;
        sensorTask();
    }

    // -----------------------------------------------------------
    // Control schedule (~150 Hz)
    // -----------------------------------------------------------
    if ((uint32_t)(loopStartUs - lastLoopUs) < LOOP_PERIOD_US) {
        // Nothing else to do this tick; return quickly so the sensor
        // schedule can fire on the next cycle without extra latency.
        return;
    }
    lastLoopUs = loopStartUs;

    nowUs = loopStartUs;
    nowMs = millis();

    // -----------------------------------------------------------
    // 1) Read RC: scaled + raw (atomic snapshot on 8-bit AVR).
    // -----------------------------------------------------------
    noInterrupts();
    rollPct     = receiver.getRoll();
    pitchPct    = receiver.getPitch();
    throttlePct = receiver.getThrottle();
    rollUs      = (uint16_t)receiver.getRawRoll();
    pitchUs     = (uint16_t)receiver.getRawPitch();
    throttleUs  = (uint16_t)receiver.getRawThrottle();
    interrupts();

    // -----------------------------------------------------------
    // 2) Validate signal
    // -----------------------------------------------------------
    rawValid =
        rawLooksValid(rollUs) &&
        rawLooksValid(pitchUs) &&
        rawLooksValid(throttleUs);

    if (rawValid) {
        lastRxOkUs = nowUs;
    }
    timedOut =
        ((uint32_t)(nowUs - lastRxOkUs) > FAILSAFE_TIMEOUT_US);

    failsafe = (!rawValid) || timedOut || (!imuReady);

    // -----------------------------------------------------------
    // 3) Normalize inputs
    // -----------------------------------------------------------
    roll     = normBipolar(rollPct);
    pitch    = normBipolar(pitchPct);
    throttle = normUnipolar(throttlePct);

    // -----------------------------------------------------------
    // 4) Apply direction config
    // -----------------------------------------------------------
    roll  *= ROLL_DIR;
    pitch *= PITCH_DIR;

    // -----------------------------------------------------------
    // 5) FAILSAFE
    // -----------------------------------------------------------
    if (failsafe) {
        if (armed) rearmLockout = true;
        armed            = false;
        throttleLowSince = 0;
        disarmStickSince = 0;
        applySafeOutputs();
    } else {
        inStartupHold =
            ((uint32_t)(nowMs - bootMs) < SAFE_STARTUP_HOLD_MS);

        if (inStartupHold) {
            armed            = false;
            throttleLowSince = 0;
            disarmStickSince = 0;
            applySafeOutputs();
        } else {
            // Manual disarm gesture
            if (throttle < ARM_THROTTLE_MAX &&
                roll     < DISARM_ROLL_THRESHOLD &&
                fabsf(pitch) < 0.2f) {
                if (disarmStickSince == 0) {
                    disarmStickSince = nowMs;
                } else if ((uint32_t)(nowMs - disarmStickSince)
                           >= DISARM_STICK_HOLD_MS) {
                    armed            = false;
                    rearmLockout     = false;
                    throttleLowSince = 0;
                    applySafeOutputs();
                }
            } else {
                disarmStickSince = 0;
            }

            // Arming
            if (!armed) {
                if (rearmLockout) {
                    if (throttle > ARM_THROTTLE_MAX) {
                        rearmLockout    = false;
                        throttleLowSince = 0;
                    }
                } else if (throttle < ARM_THROTTLE_MAX) {
                    if (throttleLowSince == 0) {
                        throttleLowSince = nowMs;
                    } else if ((uint32_t)(nowMs - throttleLowSince)
                               >= ARM_STABLE_MS) {
                        armed = true;
                    }
                } else {
                    throttleLowSince = 0;
                }
            }

            // -------------------------------------------------------
            // Stabilized control (ARMED)
            //   Pipeline (separated stages):
            //     sensor read   -> imu_fusion::updateSensors (100 Hz)
            //     sensor fusion -> Kalman roll/pitch + yaw comp-filter
            //     control       -> FlightController attitudeController+SAS
            // -------------------------------------------------------
            if (!armed) {
                applySafeOutputs();
            } else {
                theta_d = pitch * MAX_ANGLE;
                phi_d   = roll  * MAX_ANGLE;
                tau_d   = throttle;

                // Feed real sensor estimates into FlightController.
                feedFlightController();

                attitude_d[0] = theta_d;
                attitude_d[1] = phi_d;
                attitude_d[2] = tau_d;

                fc.attitudeController(attitude_d, demands);
                fc.SAS(demands, controls);

                pitchCmd = clampf(controls[0] / SAT_ANGLE, -1.0f, 1.0f);
                rollCmd  = clampf(controls[1] / SAT_ANGLE, -1.0f, 1.0f);

                left  = pitchCmd + rollCmd;
                right = pitchCmd - rollCmd;

                left  = clampf(left,  -1.0f, 1.0f);
                right = clampf(right, -1.0f, 1.0f);

                leftDeg  = normToElevonDeg(left);
                rightDeg = normToElevonDeg(right);

                elevons.setPosition(leftDeg, rightDeg);

                thrUs = throttleToUs(throttle);
                pwmDriver.write_us(THROTTLE_OUT_CH, thrUs);
            }
        }
    }

    // -----------------------------------------------------------
    // Loop timing / jitter tracking
    // -----------------------------------------------------------
    loopExecUs = (uint32_t)(micros() - loopStartUs);
    if (loopExecUs < loopExecMin) loopExecMin = loopExecUs;
    if (loopExecUs > loopExecMax) loopExecMax = loopExecUs;
    loopExecAcc += loopExecUs;
    loopExecCnt += 1;

    // -----------------------------------------------------------
    // Debug output (~10 Hz)
    // -----------------------------------------------------------
    if ((uint32_t)(nowMs - lastDbgMs) >= DEBUG_PERIOD_MS) {
        lastDbgMs = nowMs;

        uint32_t avg = (loopExecCnt > 0)
                        ? (loopExecAcc / loopExecCnt) : 0;

        Serial.print(F("armed="));     Serial.print(armed        ? 1 : 0);
        Serial.print(F(" fs="));        Serial.print(failsafe    ? 1 : 0);
        Serial.print(F(" lk="));        Serial.print(rearmLockout ? 1 : 0);
        Serial.print(F(" sOk="));       Serial.print(sensorsOk    ? 1 : 0);
        Serial.print(F(" phi="));       Serial.print(s_phi,   2);
        Serial.print(F(" th="));        Serial.print(s_theta, 2);
        Serial.print(F(" psi="));       Serial.print(s_psi,   2);
        Serial.print(F(" p="));         Serial.print(s_p,     2);
        Serial.print(F(" q="));         Serial.print(s_q,     2);
        Serial.print(F(" r="));         Serial.print(s_r,     2);
        Serial.print(F(" rc(r,p,t)="));
        Serial.print(roll, 2);    Serial.print(',');
        Serial.print(pitch, 2);   Serial.print(',');
        Serial.print(throttle, 2);
        Serial.print(F(" loop(us) min/avg/max="));
        Serial.print(loopExecMin); Serial.print('/');
        Serial.print(avg);         Serial.print('/');
        Serial.println(loopExecMax);

        // Reset min/max window each debug cycle so jitter reflects
        // recent behavior, not boot-time transients.
        loopExecMin = 0xFFFFFFFFUL;
        loopExecMax = 0;
        loopExecAcc = 0;
        loopExecCnt = 0;
    }
}
