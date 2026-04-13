#include <AtabeyAutopilot.h>

using namespace atabey::drivers;
using namespace atabey::utils;

ImuSensor imu;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("IMU TEST BASLIYOR...");

    if (!imu.init()) {
        Serial.println("IMU INIT FAILED!");
        while (1);
    }

    Serial.println("IMU INIT OK");

    Serial.println("Kalibrasyon basliyor... Lutfen sabit tut.");
    imu.calibrate();
    Serial.println("Kalibrasyon tamamlandi.");
}

void loop() {
    imu.update();

    Vec3f accel = imu.getAccel();
    Vec3f gyro  = imu.getGyro();
    Vec3f mag   = imu.getMag();

    Serial.print("ACCEL: ");
    Serial.print(accel.x); Serial.print(", ");
    Serial.print(accel.y); Serial.print(", ");
    Serial.print(accel.z);

    Serial.print(" | GYRO: ");
    Serial.print(gyro.x); Serial.print(", ");
    Serial.print(gyro.y); Serial.print(", ");
    Serial.print(gyro.z);

    Serial.print(" | MAG: ");
    Serial.print(mag.x); Serial.print(", ");
    Serial.print(mag.y); Serial.print(", ");
    Serial.print(mag.z);

    Serial.println();

    delay(50);
}