#pragma once

// Tüm sensörler için ortak interface
class ISensor {
public:
    virtual ~ISensor() = default;

    // Sensörü başlatır (I2C/SPI/UART init, WHOAMI kontrolü vs.)
    virtual bool init() = 0;

    // Sensörden yeni veri okunur
    virtual void update() = 0;

    // Sensör düzgün çalışıyor mu? kontrolü
    virtual bool isHealthy() const = 0;
};
