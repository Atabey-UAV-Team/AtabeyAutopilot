#include <AtabeyAutopilot.h>

// =============================================================
// MANUAL FLIGHT TEST (Flying Wing / Elevon)
// Repo modules used (dev branch):
// - atabey::comm::Receiver            : RC PWM input (roll/pitch/throttle)
// - atabey::drivers::Timer4ServoDriver: Hardware PWM outputs (channels 0..2)
// - atabey::drivers::ServoPWM         : 2-channel surface output (perfect for elevons)
//
// No PID, no stabilization, no filtering.
// =============================================================

using namespace atabey::comm;
using namespace atabey::drivers;
using namespace atabey::utils;

// -------------------- Instances --------------------
Receiver receiver;
Timer4ServoDriver pwmDriver;

// ServoPWM is a 2-channel helper; we use it for LEFT and RIGHT elevon outputs.
// Channel mapping inside Timer4ServoDriver (see ServoDriver.h):
//   channel 0 -> OC4A (pin 6)
//   channel 1 -> OC4B (pin 7)
//   channel 2 -> OC4C (pin 8)
ServoPWM<Timer4ServoDriver> elevons(pwmDriver, 0, 1);

// Throttle output uses Timer4ServoDriver directly (pick an available channel).
// NOTE: Timer4ServoDriver only has channels 0..2. Since elevons already use 0 and 1,
// we use channel 2 for throttle (ESC).
#define THROTTLE_OUT_CH 2

// -------------------- Safety / RC signal loss heuristic --------------------
// Receiver has no explicit "signal lost" API, so we detect loss by:
// - raw PWM being out of plausible range, OR
// - no valid raw updates for a timeout window
#define RX_RAW_MIN_US 900
#define RX_RAW_MAX_US 2100
#define RX_TIMEOUT_US 250000 // 250 ms

static uint32_t lastRxOkMicros = 0;

// Debug print period (optional)
#define DEBUG_PERIOD_MS 100

// -------------------- Helpers (no dynamic allocation) --------------------

static inline bool rawLooksValid(uint16_t us) {
  return (us >= RX_RAW_MIN_US && us <= RX_RAW_MAX_US);
}

// RC inputs come from Receiver as:
// - roll, pitch : [-100 .. 100]
// - throttle    : [0 .. 100]
static inline float normBipolarFromPercent(int16_t vPct) {
  float v = (float)vPct / 100.0f;
  return clamp(v, -1.0f, 1.0f);
}

static inline float normThrottleFromPercent(int16_t vPct) {
  float v = (float)vPct / 100.0f;
  return clamp(v, 0.0f, 1.0f);
}

// Convert normalized surface command [-1..1] into elevon angle in degrees [-20..20].
// ServoPWM internally clamps to ELEVON_MIN_ANGLE/ELEVON_MAX_ANGLE (see servo.h/servo.tpp).
static inline float normToElevonAngleDeg(float x) {
  x = clamp(x, -1.0f, 1.0f);
  return x * 20.0f;
}

// Convert throttle [0..1] to ESC microseconds [1000..2000]
static inline uint16_t throttleToUs(float t) {
  t = clamp(t, 0.0f, 1.0f);
  return (uint16_t)(1000u + (uint16_t)(t * 1000.0f));
}

void setup() {
  Serial.begin(115200);
  delay(50);

  Serial.println("=== Atabey MANUAL FLIGHT TEST (Flying Wing / Elevon Mix) ===");

  // Initialize RC input (interrupt-based in Receiver)
  receiver.init();

  // Initialize PWM outputs for elevons (this calls pwmDriver.init() and sets trim)
  elevons.init();

  // Initialize throttle output to safe low (ESC disarm)
  pwmDriver.write_us(THROTTLE_OUT_CH, 1000);

  lastRxOkMicros = micros();

  Serial.println("Init done.");
}

void loop() {
  // --------------------
  // 1) Read RC inputs
  // --------------------
  const int16_t rollPct     = receiver.getRoll();      // [-100..100]
  const int16_t pitchPct    = receiver.getPitch();     // [-100..100]
  const int16_t throttlePct = receiver.getThrottle();  // [0..100]

  // Also read raw pulse widths for signal-loss detection
  const uint16_t rollUs     = (uint16_t)receiver.getRawRoll();
  const uint16_t pitchUs    = (uint16_t)receiver.getRawPitch();
  const uint16_t throttleUs = (uint16_t)receiver.getRawThrottle();

  const bool rawValid =
      rawLooksValid(rollUs) &&
      rawLooksValid(pitchUs) &&
      rawLooksValid(throttleUs);

  if (rawValid) {
    lastRxOkMicros = micros();
  }

  const bool signalLost = ((uint32_t)(micros() - lastRxOkMicros) > RX_TIMEOUT_US);

  // --------------------
  // 2) Normalize inputs
  // --------------------
  // roll, pitch -> [-1..1]
  // throttle    -> [0..1]
  float roll     = normBipolarFromPercent(rollPct);
  float pitch    = normBipolarFromPercent(pitchPct);
  float throttle = normThrottleFromPercent(throttlePct);

  // --------------------
  // 3) SAFETY: RC lost -> neutral + throttle cut
  // --------------------
  if (signalLost) {
    roll = 0.0f;
    pitch = 0.0f;
    throttle = 0.0f;
  }

  // =============================================================
  // 4) ELEVON MIXING (CORE CONCEPT)
  //
  // Flying wing aircraft typically have no separate aileron/elevator.
  // Instead, each wing has an "elevon" surface that combines:
  // - Elevator (pitch control): both elevons move the same direction
  // - Aileron (roll control) : elevons move opposite directions
  //
  // Mix formula (normalized commands):
  //   left_elevon  = elevator + aileron
  //   right_elevon = elevator - aileron
  //
  // Here:
  //   elevator = pitch
  //   aileron  = roll
  // =============================================================
  float leftElevon  = pitch + roll;
  float rightElevon = pitch - roll;

  // Clamp mixed commands to valid surface range [-1..1]
  leftElevon  = clamp(leftElevon,  -1.0f, 1.0f);
  rightElevon = clamp(rightElevon, -1.0f, 1.0f);

  // Convert surface commands to angle range [-20..20] degrees
  const float leftAngleDeg  = normToElevonAngleDeg(leftElevon);
  const float rightAngleDeg = normToElevonAngleDeg(rightElevon);

  // --------------------
  // 5) Write outputs
  // --------------------
  // Elevons via ServoPWM:
  //   left  -> channel 0
  //   right -> channel 1
  elevons.setPosition(leftAngleDeg, rightAngleDeg);

  // Throttle via Timer4ServoDriver directly in microseconds [1000..2000]
  const uint16_t throttleOutUs = throttleToUs(throttle);
  pwmDriver.write_us(THROTTLE_OUT_CH, throttleOutUs);

  // --------------------
  // 6) Optional debug output
  // --------------------
  static uint32_t lastDbgMs = 0;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastDbgMs) >= DEBUG_PERIOD_MS) {
    lastDbgMs = nowMs;

    Serial.print(signalLost ? "RX LOST" : "RX OK  ");
    Serial.print(" | roll=");
    Serial.print(roll, 2);
    Serial.print(" pitch=");
    Serial.print(pitch, 2);
    Serial.print(" thr=");
    Serial.print(throttle, 2);

    Serial.print(" | L=");
    Serial.print(leftElevon, 2);
    Serial.print(" R=");
    Serial.print(rightElevon, 2);

    Serial.print(" | thr_us=");
    Serial.println(throttleOutUs);
  }

  // Keep loop real-time friendly: no delay().
}