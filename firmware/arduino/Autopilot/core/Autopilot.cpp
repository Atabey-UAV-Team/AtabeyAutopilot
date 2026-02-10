#include "Autopilot.h"

// Donanım Driverleri
#include "../drivers/imu/imu.h"
#include "../drivers/gps/gps.h"
#include "../drivers/servo/servo.h"

void Autopilot::begin() {

    lastMs = millis();
    dt = 0.01f;

    roll = pitch = yaw = 0.0f;
    rollRate = pitchRate = yawRate = 0.0f;

    desiredRoll = 0.0f;
    desiredPitch = 0.0f;

    aileron = elevator = rudder = throttle = 0.0f;

}

void Autopilot::update() {
    updateTime();
    /* readSensors();
    estimateAttitude();
    pidControl();
    updateActuators*/
}

void Autopilot::updateTime() {
    
    unsigned long int now = millis();
    dt = now - lastMs;
    lastMs = now;

    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }
}

void Autopilot::readSensors() {}

void Autopilot::estimateAttitude() {
    roll  += rollRate  * dt;
    pitch += pitchRate * dt;
    yaw   += yawRate   * dt;
}

void Autopilot::pidControl() {}

void Autopilot::updateActuators() {
    // Servo kütüphanesi böyle yazılır tahmini
    // Servos_write(aileron, elevator, rudder, throttle);
}
