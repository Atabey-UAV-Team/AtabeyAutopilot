// =============================================================
// PIN WIRING TABLE
// =============================================================
// Function        MCU Pin        Hardware
// -------------------------------------------------------------
// LEFT ELEVON     D6 (OC4A)      Servo Left Elevon
// RIGHT ELEVON    D7 (OC4B)      Servo Right Elevon
// THROTTLE (ESC)  D8 (OC4C)      ESC Signal
// ROLL INPUT      D2             Roll Signal
// PITCH INPUT     D3             Pitch Signal
// THROTTLE INPUT  D18            Throttle Signal
// =============================================================

#include <AtabeyAutopilot.h>
#include <math.h>   // fabsf() for disarm-gesture pitch check

// =============================================================
// ucus_test.ino
// MANUAL FLIGHT TEST (Flying Wing / Elevon Mix) - SAFETY FOCUSED
//
// Uses ONLY existing dev-branch modules:
//   - atabey::comm::Receiver
//   - atabey::drivers::ServoPWM
//   - atabey::drivers::Timer4ServoDriver
//
// NO PID, NO stabilization, NO filtering, NO dynamic allocation,
// NO blocking delays in the main loop.
// =============================================================

using namespace atabey::comm;
using namespace atabey::drivers;
using namespace atabey::control;

// -------------------------------------------------------------
// Attitude stabilization limits
// -------------------------------------------------------------
// RC stick deflection -> max commanded attitude angle (rad). ~20 deg.
#define MAX_ANGLE        0.35f
// Must match FlightController::saturationAngle (rad).
#define SAT_ANGLE        0.3491f

// -------------------------------------------------------------
// Direction configuration (applied BEFORE mixing).
// Flip signs here if a surface responds in the wrong direction.
// -------------------------------------------------------------
#define ROLL_DIR   1.0f
#define PITCH_DIR  1.0f

// -------------------------------------------------------------
// Hardware channel mapping (Timer4ServoDriver):
//   channel 0 -> OC4A (pin 6)  -> LEFT  elevon
//   channel 1 -> OC4B (pin 7)  -> RIGHT elevon
//   channel 2 -> OC4C (pin 8)  -> THROTTLE (ESC)
// -------------------------------------------------------------
#define THROTTLE_OUT_CH 2

// -------------------------------------------------------------
// Safety timings
// -------------------------------------------------------------
#define SAFE_STARTUP_HOLD_MS    3000UL   // hold throttle-cut 3s after boot
#define ARM_STABLE_MS           2000UL   // throttle-low stable duration to arm
#define FAILSAFE_TIMEOUT_US   100000UL   // >100 ms without valid RC -> failsafe

// -------------------------------------------------------------
// RC raw-pulse plausibility window (microseconds).
// Receiver has no explicit isValid(); we validate raw pulse widths.
// -------------------------------------------------------------
#define RX_RAW_MIN_US  900
#define RX_RAW_MAX_US  2100

// Arming: throttle must be below 5% (normalized)
#define ARM_THROTTLE_MAX 0.05f

// Loop rate target: ~150 Hz  (1 / 150 Hz ~= 6666 us)
#define LOOP_PERIOD_US 6666UL

// Debug output rate: ~10 Hz
#define DEBUG_PERIOD_MS 100UL

// -------------------------------------------------------------
// Instances
// -------------------------------------------------------------
Receiver           receiver;
Timer4ServoDriver  pwmDriver;

// ch0 = LEFT elevon, ch1 = RIGHT elevon
ServoPWM<Timer4ServoDriver> elevons(pwmDriver, 0, 1);

// Stabilization: attitude + SAS
FlightController   fc;

// -------------------------------------------------------------
// State
// -------------------------------------------------------------
static bool     armed    = false;
static bool     failsafe = true;

// SAFETY: latch set when failsafe fires while armed. Blocks auto re-arm
// when RC recovers; requires pilot to raise throttle stick to clear.
static bool     rearmLockout = false;

static uint32_t bootMs            = 0;
static uint32_t lastRxOkUs        = 0;
static uint32_t throttleLowSince  = 0;   // ms; 0 = not currently low
static uint32_t lastLoopUs        = 0;
static uint32_t lastDbgMs         = 0;

// SAFETY: manual stick-disarm gesture (throttle low + full-left roll >= 2s)
static uint32_t disarmStickSince  = 0;   // ms; 0 = gesture not currently held
#define DISARM_STICK_HOLD_MS   2000UL
#define DISARM_ROLL_THRESHOLD  -0.9f

// -------------------------------------------------------------
// Loop Working Variables (pre-allocated for safety / determinism)
// -------------------------------------------------------------
static uint32_t nowUs          = 0;
static uint32_t nowMs          = 0;

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

// Stabilization working buffers (pre-allocated, not on stack in loop)
static float    theta_d        = 0.0f;
static float    phi_d          = 0.0f;
static float    tau_d          = 0.0f;
static float    attitude_d[3]  = {0.0f, 0.0f, 0.0f};
static float    demands[3]     = {0.0f, 0.0f, 0.0f};
static float    controls[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
static float    pitchCmd       = 0.0f;
static float    rollCmd        = 0.0f;

// -------------------------------------------------------------
// Helpers (no dynamic allocation, all inline)
// -------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline bool rawLooksValid(uint16_t us) {
    return (us >= RX_RAW_MIN_US && us <= RX_RAW_MAX_US);
}

// [-100..100] -> [-1..1]
static inline float normBipolar(int16_t pct) {
    return clampf((float)pct / 100.0f, -1.0f, 1.0f);
}

// [0..100] -> [0..1]
static inline float normUnipolar(int16_t pct) {
    return clampf((float)pct / 100.0f, 0.0f, 1.0f);
}

// [-1..1] -> [-20..20] deg
static inline float normToElevonDeg(float x) {
    return clampf(x, -1.0f, 1.0f) * 20.0f;
}

// [0..1] -> [1000..2000] us
static inline uint16_t throttleToUs(float t) {
    t = clampf(t, 0.0f, 1.0f);
    return (uint16_t)(1000.0f + t * 1000.0f);
}

// Force immediate safe outputs: elevons neutral + throttle cut.
static inline void applySafeOutputs() {
    elevons.setPosition(0.0f, 0.0f);
    pwmDriver.write_us(THROTTLE_OUT_CH, 1000);
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
    elevons.init();   // initializes pwmDriver and trims ch0/ch1

    // Immediately enforce safe outputs during startup hold.
    applySafeOutputs();

    bootMs           = millis();
    lastRxOkUs       = micros();
    lastLoopUs       = micros();
    lastDbgMs        = millis();
    throttleLowSince = 0;

    armed    = false;
    failsafe = true;
}

// =============================================================
// loop() - non-blocking, ~150 Hz cadence
// =============================================================
void loop() {
    nowUs = micros();

    // -------- Rate limit to ~150 Hz (no delay()) --------
    if ((uint32_t)(nowUs - lastLoopUs) < LOOP_PERIOD_US) {
        return;
    }
    lastLoopUs = nowUs;

    nowMs = millis();

    // -----------------------------------------------------------
    // 1) Read RC: scaled + raw
    //    SAFETY: Receiver stores pulses in 16-bit volatile ints written
    //    by ISRs. On 8-bit AVR a main-context read is NOT atomic and may
    //    be torn mid-write, producing garbage that can slip through the
    //    plausibility window. Capture a consistent snapshot with
    //    interrupts disabled, then release them immediately.
    // -----------------------------------------------------------
    noInterrupts();
    rollPct     = receiver.getRoll();        // [-100..100]
    pitchPct    = receiver.getPitch();       // [-100..100]
    throttlePct = receiver.getThrottle();    // [0..100]
    rollUs      = (uint16_t)receiver.getRawRoll();
    pitchUs     = (uint16_t)receiver.getRawPitch();
    throttleUs  = (uint16_t)receiver.getRawThrottle();
    interrupts();

    // -----------------------------------------------------------
    // 2) Validate signal: raw-pulse plausibility + >100ms timeout
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

    failsafe = (!rawValid) || timedOut;

    // -----------------------------------------------------------
    // 3) Normalize inputs
    // -----------------------------------------------------------
    roll     = normBipolar(rollPct);       // [-1..1]
    pitch    = normBipolar(pitchPct);      // [-1..1]
    throttle = normUnipolar(throttlePct);  // [0..1]

    // -----------------------------------------------------------
    // 4) Apply direction config BEFORE mixing
    // -----------------------------------------------------------
    roll  *= ROLL_DIR;
    pitch *= PITCH_DIR;

    // -----------------------------------------------------------
    // 5) FAILSAFE: disarm + safe outputs, skip the rest
    // -----------------------------------------------------------
    if (failsafe) {
        // SAFETY: if we were armed when failsafe fired, latch a lockout so
        // the aircraft cannot silently re-arm the moment RC recovers.
        if (armed) rearmLockout = true;
        armed            = false;
        throttleLowSince = 0;
        disarmStickSince = 0;
        applySafeOutputs();
    } else {
        // -------------------------------------------------------
        // 6) Startup hold: 3s throttle-cut, arming blocked
        // -------------------------------------------------------
        inStartupHold =
            ((uint32_t)(nowMs - bootMs) < SAFE_STARTUP_HOLD_MS);

        if (inStartupHold) {
            armed            = false;
            throttleLowSince = 0;
            disarmStickSince = 0;
            applySafeOutputs();
        } else {
            // ---------------------------------------------------
            // 6b) Manual stick-disarm (runs regardless of armed state).
            //     Gesture: throttle <5% AND full-left roll (<-0.9)
            //     held for >=2s -> force disarm. Pilot-intentional,
            //     so we also reset the arming timer so it cannot
            //     re-arm immediately on release.
            // ---------------------------------------------------
            // SAFETY: also require pitch near neutral so this gesture
            // cannot be entered accidentally during aggressive maneuvers.
            if (throttle < ARM_THROTTLE_MAX &&
                roll     < DISARM_ROLL_THRESHOLD &&
                fabsf(pitch) < 0.2f) {
                if (disarmStickSince == 0) {
                    disarmStickSince = nowMs;
                } else if ((uint32_t)(nowMs - disarmStickSince)
                           >= DISARM_STICK_HOLD_MS) {
                    armed            = false;
                    rearmLockout     = false;   // pilot-initiated: allow re-arm
                    throttleLowSince = 0;       // block instant re-arm on release

                    applySafeOutputs();
                }
            } else {
                disarmStickSince = 0;
            }

            // ---------------------------------------------------
            // 7) Arming: throttle < 5% stable for >= 2000 ms
            //    Start DISARMED; stay armed once set (until
            //    failsafe/reboot).
            // ---------------------------------------------------
            if (!armed) {
                // SAFETY: after a failsafe event the lockout blocks
                // arming until the pilot actively raises the throttle
                // stick (explicit acknowledgement). Only then does the
                // normal throttle-low-stable arming path resume.
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

            // ---------------------------------------------------
            // 8) Stabilized control (only when ARMED).
            //    Not armed -> neutral surfaces + throttle cut.
            //
            //    Flow: RC sticks -> desired attitude -> FlightController
            //          -> normalized elevon cmds -> existing mixing.
            // ---------------------------------------------------
            if (!armed) {
                applySafeOutputs();
            } else {
                // RC -> desired attitude (rad) + throttle passthrough
                theta_d = pitch * MAX_ANGLE;
                phi_d   = roll  * MAX_ANGLE;
                tau_d   = throttle;

                // SIMPLIFIED sensor inputs: only attitude angles are fed
                // from RC-normalized roll/pitch until the real IMU is
                // integrated. Rates, body velocities, and position = 0.
                fc.updateSensors(
                    0.0f, 0.0f, 0.0f,      // PN, PE, h
                    0.0f, 0.0f, 0.0f,      // u,  v,  w
                    0.0f, 0.0f, 0.0f,      // p,  q,  r
                    roll, pitch, 0.0f);    // phi, theta, psi

                attitude_d[0] = theta_d;
                attitude_d[1] = phi_d;
                attitude_d[2] = tau_d;

                fc.attitudeController(attitude_d, demands);
                fc.SAS(demands, controls);

                // controls[0]=elevator (pitch), controls[1]=aileron (roll).
                // Normalize by saturationAngle, clamp hard to [-1,1].
                pitchCmd = clampf(controls[0] / SAT_ANGLE, -1.0f, 1.0f);
                rollCmd  = clampf(controls[1] / SAT_ANGLE, -1.0f, 1.0f);

                // ----- EXISTING elevon mixing (unchanged) -----
                left  = pitchCmd + rollCmd;
                right = pitchCmd - rollCmd;

                left  = clampf(left,  -1.0f, 1.0f);
                right = clampf(right, -1.0f, 1.0f);

                leftDeg  = normToElevonDeg(left);
                rightDeg = normToElevonDeg(right);

                elevons.setPosition(leftDeg, rightDeg);

                // Throttle output only when ARMED.
                thrUs = throttleToUs(throttle);
                pwmDriver.write_us(THROTTLE_OUT_CH, thrUs);
            }
        }
    }

    // -----------------------------------------------------------
    // 10) Debug output (~10 Hz)
    // -----------------------------------------------------------
    if ((uint32_t)(nowMs - lastDbgMs) >= DEBUG_PERIOD_MS) {
        lastDbgMs = nowMs;

        Serial.print(F("armed="));     Serial.print(armed        ? 1 : 0);
        Serial.print(F(" failsafe="));  Serial.print(failsafe    ? 1 : 0);
        Serial.print(F(" lockout="));   Serial.print(rearmLockout ? 1 : 0);
        Serial.print(F(" roll="));      Serial.print(roll,     2);
        Serial.print(F(" pitch="));     Serial.print(pitch,    2);
        Serial.print(F(" thr="));       Serial.print(throttle, 2);
        Serial.println();
    }
}
