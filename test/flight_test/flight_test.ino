#include <AtabeyAutopilot.h>

// =============================================================
// MANUAL FLIGHT TEST (Flying Wing / Elevon Mix) - SAFETY FOCUSED
//
// Output layer: Serial3 (UART) only. No hardware PWM.
// Actuator line format (per loop):
//   <L:1500,R:1500,T:1000>\n
// =============================================================

using namespace atabey::comm;

// -------------------------------------------------------------
// Direction configuration (VERY IMPORTANT)
// Flip signs here if surfaces respond in the wrong direction.
// -------------------------------------------------------------
#define ROLL_DIR  1.0f
#define PITCH_DIR 1.0f

// -------------------------------------------------------------
// Safety timings
// -------------------------------------------------------------
#define SAFE_STARTUP_HOLD_MS 3000 // hold throttle cut for 3s after boot
#define ARM_STABLE_MS 2000 // throttle low stable time required to arm
#define FAILSAFE_TIMEOUT_US 100000 // 100 ms max since last valid RC

// -------------------------------------------------------------
// RC validity (raw PWM microseconds) heuristic
// -------------------------------------------------------------
#define RX_RAW_MIN_US 900
#define RX_RAW_MAX_US 2100

// Arming threshold: throttle must be < 5%
#define ARM_THROTTLE_MAX 0.05f

// Debug output rate
#define DEBUG_PERIOD_MS 100

// Serial3 actuator output baud
#define ACTUATOR_BAUD 115200

// -------------------------------------------------------------
// Instances
// -------------------------------------------------------------
Receiver receiver;

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

static uint32_t lastRxOkMicros = 0;
static uint32_t throttleLowSinceMs = 0;

// -------------------------------------------------------------
// Helpers
// -------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline uint16_t clampU16(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return (uint16_t)lo;
  if (v > hi) return (uint16_t)hi;
  return (uint16_t)v;
}

static inline bool rawLooksValid(uint16_t us) {
  return (us >= RX_RAW_MIN_US && us <= RX_RAW_MAX_US);
}

static inline float normBipolarFromPercent(int16_t vPct) {
  float v = (float)vPct / 100.0f;
  return clampf(v, -1.0f, 1.0f);
}

static inline float normThrottleFromPercent(int16_t vPct) {
  float v = (float)vPct / 100.0f;
  return clampf(v, 0.0f, 1.0f);
}

static inline float normToElevonAngleDeg(float x) {
  x = clampf(x, -1.0f, 1.0f);
  return x * 20.0f;
}

static inline uint16_t throttleToUs(float t) {
  t = clampf(t, 0.0f, 1.0f);
  return (uint16_t)(1000u + (uint16_t)(t * 1000.0f));
}

// Elevon degrees -> microseconds
//   0 deg  -> 1500us, +20 -> 2000us, -20 -> 1000us
static inline uint16_t elevonDegToUs(float deg) {
  float us = 1500.0f + (deg / 20.0f) * 500.0f;
  return clampU16((int32_t)(us + 0.5f), 1000, 2000);
}

// -------------------------------------------------------------
// Output module: Serial3 actuator line
// -------------------------------------------------------------
static inline void writeOutputs(float leftDeg, float rightDeg, uint16_t throttleUsOut) {
  const uint16_t lUs = elevonDegToUs(leftDeg);
  const uint16_t rUs = elevonDegToUs(rightDeg);
  const uint16_t tUs = clampU16((int32_t)throttleUsOut, 1000, 2000);

  char buf[40];
  // Format: <L:1500,R:1500,T:1000>\n
  int n = snprintf(buf, sizeof(buf), "<L:%u,R:%u,T:%u>\n",
                   (unsigned)lUs, (unsigned)rUs, (unsigned)tUs);
  if (n > 0) {
    Serial3.write((const uint8_t*)buf, (size_t)n);
  }
}

static inline void writeSafeOutputs() {
  // Neutral elevons + throttle cut
  writeOutputs(0.0f, 0.0f, 1000);
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(ACTUATOR_BAUD);
  delay(50);

  Serial.println("=== SAFE MANUAL FLIGHT TEST (Flying Wing / Elevon) ===");
  Serial.println("Booting DISARMED. Throttle is held at 1000us during startup hold.");
  Serial.println("Actuator output: Serial3 @115200, format <L:us,R:us,T:us>\\n");

  receiver.init();

  bootMs = millis();
  lastRxOkMicros = micros();

  armed = false;
  failsafe = true;
  throttleLowSinceMs = 0;

  // Safe startup: enforce throttle cut + neutral elevons immediately.
  writeSafeOutputs();
}

void loop() {
  nowMs = millis();
  nowUs = micros();

  // -----------------------------------------------------------
  // 1) Read RC (scaled) + raw
  // -----------------------------------------------------------
  rollPct     = receiver.getRoll();
  pitchPct    = receiver.getPitch();
  throttlePct = receiver.getThrottle();

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

  const bool timedOut = ((uint32_t)(nowUs - lastRxOkMicros) > FAILSAFE_TIMEOUT_US);

  failsafe = (!rawValid) || timedOut;

  // -----------------------------------------------------------
  // 3) Normalize inputs
  // -----------------------------------------------------------
  roll     = normBipolarFromPercent(rollPct);
  pitch    = normBipolarFromPercent(pitchPct);
  throttle = normThrottleFromPercent(throttlePct);

  // -----------------------------------------------------------
  // 4) Apply direction config BEFORE mixing
  // -----------------------------------------------------------
  roll  *= ROLL_DIR;
  pitch *= PITCH_DIR;

  // -----------------------------------------------------------
  // 5) Failsafe action (MANDATORY)
  // -----------------------------------------------------------
  if (failsafe) {
    armed = false;
    throttleLowSinceMs = 0;
    writeSafeOutputs();
  } else {
    // ---------------------------------------------------------
    // 6) Safe startup hold
    // ---------------------------------------------------------
    inStartupHold = (uint32_t)(nowMs - bootMs) < SAFE_STARTUP_HOLD_MS;

    if (inStartupHold) {
      armed = false;
      throttleLowSinceMs = 0;
      writeSafeOutputs();
    } else {
      // -------------------------------------------------------
      // 7) Arming logic
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
      // 8) Elevon mixing (EXACT equations)
      //   left  = pitch + roll
      //   right = pitch - roll
      // -------------------------------------------------------
      float left  = pitch + roll;
      float right = pitch - roll;

      left  = clampf(left,  -1.0f, 1.0f);
      right = clampf(right, -1.0f, 1.0f);

      const float leftDeg  = normToElevonAngleDeg(left);
      const float rightDeg = normToElevonAngleDeg(right);

      // -------------------------------------------------------
      // 9) Throttle: only applied when ARMED, otherwise 1000us
      // -------------------------------------------------------
      const uint16_t thrUs = armed ? throttleToUs(throttle) : 1000;

      // Single output write per loop
      writeOutputs(leftDeg, rightDeg, thrUs);
    }
  }

  // -----------------------------------------------------------
  // 10) Debug output (rate-limited, USB Serial only)
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

  // No delay() in loop: keep timing non-blocking.
}
