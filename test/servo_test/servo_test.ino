#include <AtabeyAutopilot.h>

using namespace atabey::drivers;

Timer4ServoDriver servoDriver;
ServoPWM<Timer4ServoDriver> elevon(servoDriver, 0, 1);

void setup() {
    elevon.init();
}

void loop() {
    elevon.setPosition(10, -10);
    delay(2000);
    elevon.setPosition(0, 0);
    delay(2000);
    elevon.setPosition(-10, 10);
    delay(2000);
    elevon.setPosition(0, 0);
    delay(2000);
}