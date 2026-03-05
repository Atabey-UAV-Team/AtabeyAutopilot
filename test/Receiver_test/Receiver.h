#ifndef RECEIVER_H
#define RECEIVER_H

class Receiver {
private:
  static const int ROLL_PIN;
  static const int PITCH_PIN;
  static const int THROTTLE_PIN;
  static const int YAW_PIN;
  
  static const int MIN_PWM;
  static const int MAX_PWM;
  
  static volatile int rollValue;
  static volatile int pitchValue;
  static volatile int throttleValue;
  static volatile int yawValue;
  
  static volatile unsigned long rollStart;
  static volatile unsigned long pitchStart;
  static volatile unsigned long throttleStart;
  static volatile unsigned long yawStart;
  
  static void rollISR();
  static void pitchISR();
  static void throttleISR();
  static void yawISR();

public:
  Receiver();
  void init();
  
  int getRoll();
  int getPitch();
  int getThrottle();
  int getYaw();
  
  int getRawRoll();
  int getRawPitch();
  int getRawThrottle();
  int getRawYaw();
};

#endif