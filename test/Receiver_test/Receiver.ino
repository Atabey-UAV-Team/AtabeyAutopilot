#include "Receiver.h"

Receiver Receiver;

void setup() {
  Serial.begin(9600);
  receiver.init();
  
  Serial.println("=== FS-i6X 4-Channel Controller ===");
  Serial.println("Roll | Pitch | Throttle | Yaw");
}

void loop() {
  int roll = receiver.getRoll();
  int pitch = receiver.getPitch();
  int throttle = receiver.getThrottle();
  int yaw = receiver.getYaw();
  
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(throttle);
  Serial.print("\t");
  Serial.println(yaw);
  
  delay(100);
}