#include "IMU.h"

// obje oluşturma (SPI, CS pin 10)
Mpu9250 imu(SPI, 10);

// Veri yapısı
ImuSample sample;
ImuConfig config;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // Konfigürasyon ayarla
    config.accel_range = 1;      // ±4G (A_4G)
    config.gyro_range = 1;       // ±500 DPS (G_500DPS)
    config.dlpf = 4;             // 20Hz (DLPF_20HZ)
    config.sample_rate_hz = 100; // 100Hz örnekleme
    config.timeout_us = 100000;  // 100ms timeout
    
    // IMU'yu başlat
    if (!imu.init(config)) {
        while(1) delay(1000); // Hata durumunda bekle (baslatma hatası)
    }
    
    // Kalibrasyon için 
    delay(2000);
    
    ImuCalib calib;
    // Kütüphanenin kendi kalibrasyonunu kullan
    calib.accel_bias_mps2 = Vec3f(0, 0, 0);// bias yok kabul ettik daha sonra ayarlanacak
    calib.gyro_bias_rads = Vec3f(0, 0, 0);
    calib.accel_scale = Vec3f(1, 1, 1);//secilen accel_range icin 
    calib.gyro_scale = Vec3f(1, 1, 1);
    
    imu.setCalib(calib);
}

void loop() {
    uint32_t now = micros(); // Mikrosaniye
    
    // Veriyi oku
    if (imu.read(sample, now)) {
               
            // Autopilot burada sample değişkenini kullanabilir            
        }//else: veri okunamadı, acil durum
    
    
    delay(10); // 100Hz için
}