#include <AtabeyAutopilot.h>
#include <Servo.h>

// =============================================================
// MANUAL FLIGHT TEST (Flying Wing / Elevon Mix) - SAFETY FOCUSED
//
// Output:
//   - Elevons: Serial3 as degrees, format  L+020R-020\n
//   - Throttle: local hardware PWM via Servo on THROTTLE_PIN
// =============================================================

using namespace atabey::comm;

// -------------------------------------------------------------
// Direction configuration (VERY IMPORTANT)
// -------------------------------------------------------------
#define ROLL_DIR  1.0f
#define PITCH_DIR 1.0f

// -------------------------------------------------------------
// Safety timings
// -------------------------------------------------------------
#define SAFE_STARTUP_HOLD_MS 3000
#define ARM_STABLE_MS        2000
#define FAILSAFE_TIMEOUT_US  100000

// -------------------------------------------------------------
// RC validity
// -------------------------------------------------------------
#define RX_RAW_MIN_US 900
#define RX_RAW_MAX_US 2100

#define ARM_THROTTLE_MAX 0.05f
#define DEBUG_PERIOD_MS  100
#define ACTUATOR_BAUD    115200

// Throttle PWM output pin
#define THROTTLE_PIN 9

// -------------------------------------------------------------
// Instances
// -------------------------------------------------------------
Receiver receiver;
Servo    throttleServo;

// -------------------------------------------------------------
// State
// -------------------------------------------------------------
static bool armed = false;
static bool failsafe = true;

static uint32_t bootMs = 0;
static int32_t  nowMs = 0;
static int32_t  nowUs = 0;
bool inStartupHold;

int16_t  rollPct, pitchPct, throttlePct;
uint16_t rollUs, pitchUs, throttleUs;
float    roll, pitch, throttle;

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

static inline bool rawLooksValid(uint16_t us) {
  return (us >= RX_RAW_MIN_US && us <= RX_RAW_MAX_US);
}

static inline float normBipolarFromPercent(int16_t vPct) {
  return clampf((float)vPct / 100.0f, -1.0f, 1.0f);
}

static inline float normThrottleFromPercent(int16_t vPct) {
  return clampf((float)vPct / 100.0f, 0.0f, 1.0f);
}

static inline float normToElevonAngleDeg(float x) {
  x = clampf(x, -1.0f, 1.0f);
  return x * 20.0f;
}

// -------------------------------------------------------------
// Output: elevons over Serial3 in DEGREES
//   Format: L<sdeg>R<sdeg>\n   e.g. L+020R-020\n
// -------------------------------------------------------------
void sendElevons(float leftDeg, float rightDeg) {
  leftDeg  = clampf(leftDeg,  -20.0f, 20.0f);
  rightDeg = clampf(rightDeg, -20.0f, 20.0f);

  int l = (int)(leftDeg  >= 0 ? leftDeg  + 0.5f : leftDeg  - 0.5f);
  int r = (int)(rightDeg >= 0 ? rightDeg + 0.5f : rightDeg - 0.5f);

  char buf[24];
  int n = snprintf(buf, sizeof(buf), "L%+04dR%+04d\n", l, r);
  if (n > 0) {
    Serial3.write((const uint8_t*)buf, (size_t)n);
  }
}

// -------------------------------------------------------------
// Output: throttle via local PWM
//   throttle [0..1] -> 1000..2000 us
// -------------------------------------------------------------
void writeThrottle(float t) {
  t = clampf(t, 0.0f, 1.0f);
  uint16_t throttleUsOut = (uint16_t)(1000.0f + t * 1000.0f);
  throttleServo.writeMicroseconds(throttleUsOut);
}

static inline void writeSafeOutputs() {
  sendElevons(0.0f, 0.0f);
  throttleServo.writeMicroseconds(1000);
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(ACTUATOR_BAUD);

  throttleServo.attach(THROTTLE_PIN);
  throttleServo.writeMicroseconds(1000);

  delay(50);

  Serial.println("=== SAFE MANUAL FLIGHT TEST (Flying Wing / Elevon) ===");
  Serial.println("Booting DISARMED. Throttle held at 1000us during startup hold.");
  Serial.println("Elevons: Serial3 @115200, format L+ddd R+ddd (degrees)");
  Serial.println("Throttle: local PWM via Servo on pin 9");

  receiver.init();

  bootMs = millis();
  lastRxOkMicros = micros();

  armed = false;
  failsafe = true;
  throttleLowSinceMs = 0;

  writeSafeOutputs();
}

void loop() {
  nowMs = millis();
  nowUs = micros();

  // 1) Read RC
  rollPct     = receiver.getRoll();
  pitchPct    = receiver.getPitch();
  throttlePct = receiver.getThrottle();

  rollUs     = (uint16_t)receiver.getRawRoll();
  pitchUs    = (uint16_t)receiver.getRawPitch();
  throttleUs = (uint16_t)receiver.getRawThrottle();

  // 2) Validate signal
  const bool rawValid =
      rawLooksValid(rollUs) &&
      rawLooksValid(pitchUs) &&
      rawLooksValid(throttleUs);

  if (rawValid) {
    lastRxOkMicros = nowUs;
  }

  const bool timedOut = ((uint32_t)(nowUs - lastRxOkMicros) > FAILSAFE_TIMEOUT_US);
  failsafe = (!rawValid) || timedOut;

  // 3) Normalize
  roll     = normBipolarFromPercent(rollPct);
  pitch    = normBipolarFromPercent(pitchPct);
  throttle = normThrottleFromPercent(throttlePct);

  // 4) Direction
  roll  *= ROLL_DIR;
  pitch *= PITCH_DIR;

  // 5) Failsafe
  if (failsafe) {
    armed = false;
    throttleLowSinceMs = 0;
    writeSafeOutputs();
  } else {
    // 6) Startup hold
    inStartupHold = (uint32_t)(nowMs - bootMs) < SAFE_STARTUP_HOLD_MS;

    if (inStartupHold) {
      armed = false;
      throttleLowSinceMs = 0;
      writeSafeOutputs();
    } else {
      // 7) Arming
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

      // 8) Elevon mixing
      float left  = pitch + roll;
      float right = pitch - roll;

      left  = clampf(left,  -1.0f, 1.0f);
      right = clampf(right, -1.0f, 1.0f);

      const float leftDeg  = normToElevonAngleDeg(left);
      const float rightDeg = normToElevonAngleDeg(right);

      // 9) Outputs
      sendElevons(leftDeg, rightDeg);
      if (armed) {
        writeThrottle(throttle);
      } else {
        throttleServo.writeMicroseconds(1000);
      }
    }
  }

  // 10) Debug (rate-limited)
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

  // No delay(): non-blocking loop.
}
