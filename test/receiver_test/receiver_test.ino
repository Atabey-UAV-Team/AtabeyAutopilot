#include <AtabeyAutopilot.h>

using namespace atabey::comm;

Receiver receiver;
int16_t dizi[3];

/* MEGA PINOUT
 * CH1: Roll - PIN 2
 * CH2: Pitch - PIN 3
 * CH3: Throttle - PIN 18
 * CH4: Yaw - PIN 19
 */

void setup() {
  Serial.begin(9600);
  receiver.init();
  
//  Serial.println("=== FS-i6X 4-Channel Controller ===");
//  Serial.println("Roll | Pitch | Throttle | Yaw");
}

void loop() {
  int16_t roll = receiver.getRoll();
  int16_t pitch = receiver.getPitch();
  int16_t throttle = receiver.getThrottle();
  int16_t yaw = receiver.getYaw();

  dizi[0] = pitch;
  dizi[1] = roll;
  dizi[2] = throttle;

//  Serial.print("Pitch: ");
//  Serial.print(pitch);
//  Serial.print("\t");
//  Serial.print("Roll: ");
//  Serial.print(roll);
//  Serial.print("\t");
//  Serial.print("Throttle: ");
//  Serial.print(throttle);
//  Serial.print("\t");
//  Serial.print("\n");
  
  Serial.write(255);

  for (int i = 0; i < 3; i++) {
    Serial.write(lowByte(dizi[i]));
    Serial.write(highByte(dizi[i]));
  }
  
  delay(10);
}
