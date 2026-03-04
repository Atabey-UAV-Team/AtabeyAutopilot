#include <AtabeyAutopilot.h>

using namespace atabey::drivers;
using namespace atabey::utils;

ImuSensor imu;

void setup() {

    Serial.begin(115200);

    if(!imu.init()) {
        Serial.println("IMU init FAILED");
        while(1);
    }

    Serial.println("IMU initialized");
}

void loop() {

    imu.update();

    Vec3f accel = imu.getAccel();
    Vec3f gyro  = imu.getGyro();

    Serial.print("ACC ");
    Serial.print(accel.x); Serial.print(" ");
    Serial.print(accel.y); Serial.print(" ");
    Serial.print(accel.z); Serial.print(" | ");

    Serial.print("GYRO ");
    Serial.print(gyro.x); Serial.print(" ");
    Serial.print(gyro.y); Serial.print(" ");
    Serial.println(gyro.z);

    delay(50);
}