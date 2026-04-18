#include <AtabeyAutopilot.h>

// =============================================================
// MANUAL FLIGHT TEST (Flying Wing / Elevon Mix) - SAFETY FOCUSED
//
// Uses ONLY existing dev-branch modules:
//  - atabey::comm::Receiver
//  - atabey::drivers::ServoPWM
//  - atabey::drivers::Timer4ServoDriver
//
// NO PID, NO stabilization, NO filtering.
// =============================================================

using namespace atabey::comm;
using namespace atabey::drivers;

// -------------------------------------------------------------
// Direction configuration (VERY IMPORTANT)
// Flip signs here if surfaces respond in the wrong direction.
// -------------------------------------------------------------
#define ROLL_DIR  1.0f
#define PITCH_DIR 1.0f

// -------------------------------------------------------------
// Hardware mapping (Timer4ServoDriver channels):
//  channel 0 -> OC4A (pin 6)
//  channel 1 -> OC4B (pin 7)
//  channel 2 -> OC4C (pin 8)
//
// Elevons use ch0/ch1 via ServoPWM.
// Throttle (ESC) uses ch2 directly.
// -------------------------------------------------------------
#define THROTTLE_OUT_CH 2

// -------------------------------------------------------------
// Safety timings
// -------------------------------------------------------------
#define SAFE_STARTUP_HOLD_MS 3000 // hold throttle cut for 3s after boot
#define ARM_STABLE_MS 2000 // throttle low stable time required to arm
#define FAILSAFE_TIMEOUT_US 100000 // 100 ms max since last valid RC

// -------------------------------------------------------------
// RC validity (raw PWM microseconds) heuristic
// Receiver has no explicit signal-lost API; we validate raw pulses.
// Adjust limits to match your receiver if needed.
// -------------------------------------------------------------
#define RX_RAW_MIN_US 900
#define RX_RAW_MAX_US 2100

// Arming threshold: throttle must be < 5%
#define ARM_THROTTLE_MAX 0.05f

// Debug output rate
#define DEBUG_PERIOD_MS 100

// -------------------------------------------------------------
// Instances
// -------------------------------------------------------------
Receiver receiver;
Timer4ServoDriver pwmDriver;

// ServoPWM is a 2-channel helper; in this flying-wing setup:
// ch0 = LEFT elevon, ch1 = RIGHT elevon
ServoPWM<Timer4ServoDriver> elevons(pwmDriver, 0, 1);

// -------------------------------------------------------------
// State
// -------------------------------------------------------------
static bool armed = false;
static bool failsafe = true;

static uint32_t bootMs = 0;
static int32_t nowMs = 0;
static int32_t nowUs = 0;
bool inStartupHold;

int16_t rollPct, pitchPct, throttlePct;
uint16_t rollUs, pitchUs, throttleUs;
float roll, pitch, throttle;

// "Last time we saw a valid RC frame" (raw pulses plausible)
static uint32_t lastRxOkMicros = 0;

// For arming: time when throttle first became "low enough"
static uint32_t throttleLowSinceMs = 0;

// -------------------------------------------------------------
// Helpers (no dynamic allocation)
// -------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline bool rawLooksValid(uint16_t us) {
  return (us >= RX_RAW_MIN_US && us <= RX_RAW_MAX_US);
}

static inline float normBipolarFromPercent(int16_t vPct) {
  // [-100..100] -> [-1..1]
  float v = (float)vPct / 100.0f;
  return clampf(v, -1.0f, 1.0f);
}

static inline float normThrottleFromPercent(int16_t vPct) {
  // [0..100] -> [0..1]
  float v = (float)vPct / 100.0f;
  return clampf(v, 0.0f, 1.0f);
}

static inline float normToElevonAngleDeg(float x) {
  // [-1..1] -> [-20..20] degrees
  x = clampf(x, -1.0f, 1.0f);
  return x * 20.0f;
}

static inline uint16_t throttleToUs(float t) {
  // [0..1] -> [1000..2000] microseconds
  t = clampf(t, 0.0f, 1.0f);
  return (uint16_t)(1000u + (uint16_t)(t * 1000.0f));
}

static inline void applySafeOutputs() {
  // Immediately force safe outputs:
  // - elevons neutral
  // - throttle cut
  elevons.setPosition(0.0f, 0.0f);
  pwmDriver.write_us(THROTTLE_OUT_CH, 1000);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  Serial.println("=== SAFE MANUAL FLIGHT TEST (Flying Wing / Elevon) ===");
  Serial.println("Booting DISARMED. Throttle is held at 1000us during startup hold.");

  receiver.init();

  // ServoPWM::init() calls pwmDriver.init() and disarms ch0/ch1 to trim.
  elevons.init();

  bootMs = millis();
  lastRxOkMicros = micros();

  armed = false;
  failsafe = true;
  throttleLowSinceMs = 0;

  // Safe startup: enforce throttle cut + neutral elevons immediately.
  applySafeOutputs();
}

void loop() {
  nowMs = millis();
  nowUs = micros();

  // -----------------------------------------------------------
  // 1) Read RC (scaled) + raw
  // -----------------------------------------------------------
  rollPct     = receiver.getRoll();      // [-100..100]
  pitchPct    = receiver.getPitch();     // [-100..100]
  throttlePct = receiver.getThrottle();  // [0..100]

  rollUs     = (uint16_t)receiver.getRawRoll();
  pitchUs    = (uint16_t)receiver.getRawPitch();
  throttleUs = (uint16_t)receiver.getRawThrottle();

  // -----------------------------------------------------------
  // 2) Validate signal (raw PWM plausibility + timeout)
  // -----------------------------------------------------------
  const bool rawValid =
      rawLooksValid(rollUs) &&
      rawLooksValid(pitchUs) &&
      rawLooksValid(throttleUs);

  if (rawValid) {
    lastRxOkMicros = nowUs;
  }

  // Rollover-safe unsigned subtraction
  const bool timedOut = ((uint32_t)(nowUs - lastRxOkMicros) > FAILSAFE_TIMEOUT_US);

  failsafe = (!rawValid) || timedOut;

  // -----------------------------------------------------------
  // 3) Normalize inputs
  // -----------------------------------------------------------
  roll     = normBipolarFromPercent(rollPct);        // [-1..1]
  pitch    = normBipolarFromPercent(pitchPct);       // [-1..1]
  throttle = normThrottleFromPercent(throttlePct);   // [0..1]

  // -----------------------------------------------------------
  // 4) Apply direction config BEFORE mixing
  // -----------------------------------------------------------
  roll  *= ROLL_DIR;
  pitch *= PITCH_DIR;

  // -----------------------------------------------------------
  // 5) Failsafe action (MANDATORY):
  // If failsafe -> immediately DISARM and force safe outputs.
  // -----------------------------------------------------------
  if (failsafe) {
    armed = false;
    throttleLowSinceMs = 0;
    applySafeOutputs();
  } else {
    // ---------------------------------------------------------
    // 6) Safe startup hold (MANDATORY):
    // Hold throttle cut and prevent arming for first 3 seconds.
    // ---------------------------------------------------------
    inStartupHold = (uint32_t)(nowMs - bootMs) < SAFE_STARTUP_HOLD_MS;

    if (inStartupHold) {
      armed = false;
      throttleLowSinceMs = 0;
      applySafeOutputs();
    } else {
      // -------------------------------------------------------
      // 7) Arming logic (MANDATORY):
      // Start DISARMED.
      // Arm ONLY if throttle < 5% and stays stable for >= 2s.
      // -------------------------------------------------------
      if (!armed) {
        if (throttle < ARM_THROTTLE_MAX) {
          if (throttleLowSinceMs == 0) {
            throttleLowSinceMs = nowMs;
          } else if ((uint32_t)(nowMs - throttleLowSinceMs) >= ARM_STABLE_MS) {
            armed = true;
          }
        } else {
          throttleLowSinceMs = 0;
        }
      }

      // -------------------------------------------------------
      // 8) Elevon mixing (MANDATORY, exact equations):
      //
      // Elevon mixing combines:
      //  - Elevator (pitch): both surfaces move same direction
      //  - Aileron (roll) : surfaces move opposite directions
      //
      // EXACT mix:
      //   left  = pitch + roll
      //   right = pitch - roll
      // -------------------------------------------------------
      float left  = pitch + roll;
      float right = pitch - roll;

      // Clamp mixed commands to [-1..1]
      left  = clampf(left,  -1.0f, 1.0f);
      right = clampf(right, -1.0f, 1.0f);

      // Convert to angle range [-20..20] degrees
      const float leftDeg  = normToElevonAngleDeg(left);
      const float rightDeg = normToElevonAngleDeg(right);

      // Send to elevon outputs (ServoPWM: ch0=left, ch1=right)
      elevons.setPosition(leftDeg, rightDeg);

      // -------------------------------------------------------
      // 9) Throttle output:
      // Only apply throttle when ARMED, otherwise always 1000us.
      // -------------------------------------------------------
      const uint16_t thrUs = armed ? throttleToUs(throttle) : 1000;
      pwmDriver.write_us(THROTTLE_OUT_CH, thrUs);
    }
  }

  // -----------------------------------------------------------
  // 10) Debug output (rate-limited)
  // -----------------------------------------------------------
  static uint32_t lastDbgMs = 0;
  if ((uint32_t)(nowMs - lastDbgMs) >= DEBUG_PERIOD_MS) {
    lastDbgMs = nowMs;

    Serial.print("armed=");
    Serial.print(armed ? "1" : "0");
    Serial.print(" failsafe=");
    Serial.print(failsafe ? "1" : "0");

    Serial.print(" roll=");
    Serial.print(roll, 2);
    Serial.print(" pitch=");
    Serial.print(pitch, 2);
    Serial.print(" thr=");
    Serial.print(throttle, 2);

    Serial.println();
  }

  // No delay() in loop: keep timing responsive.
}