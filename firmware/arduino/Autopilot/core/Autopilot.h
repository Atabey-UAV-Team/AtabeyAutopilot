#pragma once
#include <Arduino.h>

class Autopilot {
private:
    // Zaman Kontrolü
    unsigned long lastMs;
    float dt;

    // Anlık Durum (Sensör verisi)
    float roll;
    float pitch;
    float yaw;

    // Anlık Değişim (Sensör verisi)
    float rollRate;
    float pitchRate;
    float yawRate;

    // Sonuç değerler
    float desiredRoll;
    float desiredPitch;

    // Çıkış açıları
    float aileron;
    float elevator;
    float rudder;
    float throttle;

    void updateTime();
    void readSensors();
    void estimateAttitude();
    void pidControl();
    void updateActuators();

public:
    void begin();
    void update();

};
