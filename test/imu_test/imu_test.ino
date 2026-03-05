#include <AtabeyAutopilot.h>

using namespace atabey::drivers;
using namespace atabey::utils;
using namespace atabey::estimation;

ImuSensor imu;
AttitudeEstimator estimator(imu);

void setup() {

    Serial.begin(115200);

    if(!imu.init()) {
        Serial.println("IMU init FAILED");
        while(1);
    }

    if(!estimator.init()) {
        Serial.println("Estimator init FAILED");
        while(1);
    }

    Serial.println("System initialized");
}

void loop() {
    imu.update();
    estimator.update();

    Vec3f accel = imu.getAccel();
    Vec3f gyro  = imu.getGyro();
    Vec3f attitude = estimator.getAttitude();

    Serial.print("ACC ");
    Serial.print(accel.x); Serial.print(" ");
    Serial.print(accel.y); Serial.print(" ");
    Serial.print(accel.z); Serial.print(" | ");

    Serial.print("GYRO ");
    Serial.print(gyro.x); Serial.print(" ");
    Serial.print(gyro.y); Serial.print(" ");
    Serial.println(gyro.z);

    Serial.print("ATTITUDE ");
    Serial.print(rad2deg(attitude.x)); Serial.print(" ");
    Serial.print(rad2deg(attitude.y)); Serial.print(" ");
    Serial.println(rad2deg(attitude.z));

    delay(50);
}