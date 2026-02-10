#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <SPI.h>
#include <MPU9250.h> // Bolder Flight Systems kütüphanesi


struct Vec3f {
    float x;
    float y;
    float z;
    
    Vec3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

//  ham veriler
struct ImuSample {
    uint32_t t_us;          // Zaman damgası (mikrosaniye)
    Vec3f accel_mps2;       // İvme (m/s²)
    Vec3f gyro_rads;        // Açısal hız (rad/s)
    float temp_C;           
    uint32_t status;        
};

// Kalibrasyon verileri
struct ImuCalib {
    Vec3f accel_bias_mps2;  // İvme sapması
    Vec3f gyro_bias_rads;   // Jiroskop sapması
    Vec3f accel_scale;      // İvme ölçeği
    Vec3f gyro_scale;       // Jiroskop ölçeği
};

// Sensör ayarları
struct ImuConfig {
    uint8_t accel_range;    // İvme aralığı (AccelRange enum)
    uint8_t gyro_range;     // Jiroskop aralığı (GyroRange enum)
    uint8_t dlpf;           // Dijital alçak geçiren filtre
    uint16_t sample_rate_hz;// Örnekleme hızı
    uint32_t timeout_us;    // Zaman aşımı
};

class Mpu9250 {
private:
    MPU9250 mpu;                    // MPU9250 kütüphane objesi
    
    // değişkenler (imuSensor.txt dosyasındaki )
    void* hspi;                     // SPI handle
    void* cs_port;                  // CS port
    uint16_t cs_pin;                // CS pin numarası
    
    ImuConfig cfg;                  // Konfigürasyon
    ImuCalib cal;                   // Kalibrasyon
    
    float accel_lsb2mps2;          // İvme LSB'den m/s²'ye çevirme
    float gyro_lsb2rads;           // Jiroskop LSB'den rad/s'ye çevirme
    
    uint32_t last_err;             // Son hata kodu
    uint32_t last_read_us;         // Son okuma zamanı
    
public:
    // Constructor
    Mpu9250(SPIClass &bus, uint8_t csPin) 
        : mpu(bus, csPin), cs_pin(csPin) {
        
        // Varsayılan konfigürasyon( enum şeklinde )
        cfg.accel_range = 1;  // ±4G
        cfg.gyro_range = 1;   // ±500 DPS
        cfg.dlpf = 4;         // 20Hz
        cfg.sample_rate_hz = 100;
        cfg.timeout_us = 10000;
        
        last_err = 0;
        last_read_us = 0;
    }
    
    // Sensörü başlat
    bool init(ImuConfig config) {
        cfg = config;
        
        int status = mpu.begin();
        if (status < 0) {
            last_err = status;
            return false;
        }
        
        // Kütüphanenin kendi fonksiyonlarını kullandık
        mpu.setAccelRange(static_cast<MPU9250::AccelRange>(cfg.accel_range));
        mpu.setGyroRange(static_cast<MPU9250::GyroRange>(cfg.gyro_range));
        mpu.setDlpfBandwidth(static_cast<MPU9250::DlpfBandwidth>(cfg.dlpf));
        
        // Örnekleme hızını içim
        //Gerçek Örnekleme Hızı = 1000 Hz / (1 + SRD)
        if (cfg.sample_rate_hz > 0) {
            uint8_t srd = (1000 / cfg.sample_rate_hz) - 1;
            mpu.setSrd(srd);
        }
        
        return true;
    }
    
    // Kalibrasyon ayarla
    void setCalib(ImuCalib c) {
        cal = c;
        
        // Kütüphaneye bias değerlerini set et
        mpu.setAccelCalX(cal.accel_bias_mps2.x, cal.accel_scale.x);
        mpu.setAccelCalY(cal.accel_bias_mps2.y, cal.accel_scale.y);
        mpu.setAccelCalZ(cal.accel_bias_mps2.z, cal.accel_scale.z);
        
        mpu.setGyroCalX(cal.gyro_bias_rads.x, cal.gyro_scale.x);
        mpu.setGyroCalY(cal.gyro_bias_rads.y, cal.gyro_scale.y);
        mpu.setGyroCalZ(cal.gyro_bias_rads.z, cal.gyro_scale.z);
    }
    
    // Sensörden veri oku
    bool read(ImuSample& out, uint32_t now_us) {
        // Kütüphanenin readSensor fonksiyonunu kullan
        if (!mpu.readSensor()) {
            last_err = 1; // Okuma hatası
            return false;
        }
        
        // Verileri oku
        out.t_us = now_us;
        
        out.accel_mps2.x = mpu.getAccelX_mss();
        out.accel_mps2.y = mpu.getAccelY_mss();
        out.accel_mps2.z = mpu.getAccelZ_mss();
        
        out.gyro_rads.x = mpu.getGyroX_rads();
        out.gyro_rads.y = mpu.getGyroY_rads();
        out.gyro_rads.z = mpu.getGyroZ_rads();
        
        out.temp_C = mpu.getTemperature_C();
        out.status = 0; // IMU_OK
        
        last_read_us = now_us;
        last_err = 0;//Son hata kodunu sıfırla
        
        return true;
    }
    
    
};

#endif 