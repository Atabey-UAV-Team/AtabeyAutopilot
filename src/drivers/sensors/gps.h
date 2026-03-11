#pragma once

#include <Arduino.h>
#include "ISensor.h"

namespace atabey {
    namespace drivers {

        struct GpsState {

            int32_t lat = 0; // deg * 1e7
            int32_t lon = 0; // deg * 1e7
            int32_t alt = 0; // mm

            int32_t velN = 0; // mm/s
            int32_t velE = 0; // mm/s
            int32_t velD = 0; // mm/s

            uint8_t fixType = 0;

            uint32_t lastUpdate = 0; // Son güncelleme zamanı (ms)
        };

        #pragma pack(push,1)

        struct NavPVT {

            uint32_t iTOW; // GPS zaman (ms)

            uint16_t year; // Yılı (yıl)
            uint8_t month; // Ayı (ay)
            uint8_t day; // Gününü (gün)
            uint8_t hour; // Saatini (saat)
            uint8_t min; // Dakikasını (dakika)
            uint8_t sec; // Saniyesini (saniye)

            uint8_t valid; // Tarih/saat geçerli bayrağı

            uint32_t tAcc; // Zaman doğruluğu (ns)
            int32_t nano; // Nanosecond kısmı (ns)

            uint8_t fixType; // 0 = No fix, 1 = Dead reckoning only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead reckoning combined, 5 = Time only fix
            uint8_t flags; // Fix durumunu gösteren bayraklar
            uint8_t flags2; // Ek bayraklar
            uint8_t numSV; // Görülen uydu sayısı

            int32_t lon; // Boylam (deg * 1e-7)
            int32_t lat; // Enlem (deg * 1e-7)

            int32_t height; // Anten yüksekliği (mm)
            int32_t hMSL; // Deniz seviyesine göre yükseklik (mm)

            uint32_t hAcc; // Horizontal doğruluk (mm)
            uint32_t vAcc; // Vertical doğruluk (mm)

            int32_t velN; // Kuzey yönünde hız (mm/s)
            int32_t velE; // Doğu yönünde hız (mm/s)
            int32_t velD; // Aşağı yönünde hız (mm/s)

        };

        #pragma pack(pop)
        static_assert(sizeof(NavPVT) == 92, "NavPVT yapısı 92 byte olmalıdır!");

        class GpsSensor : public atabey::drivers::ISensor {
            private:
                enum UbxState {
                    WAIT_SYNC1,
                    WAIT_SYNC2,
                    READ_CLASS,
                    READ_ID,
                    READ_LENGTH1,
                    READ_LENGTH2,
                    READ_PAYLOAD,
                    READ_CKA,
                    READ_CKB
                };

                UbxState state = WAIT_SYNC1; // UBX protokolü durum makinesi

                uint8_t msgClass; // UBX mesaj sınıfı
                uint8_t msgId; // UBX mesaj kimliği

                uint16_t length; // UBX mesaj yük uzunluğu
                uint16_t counter; // Yük okuma sayacı

                uint8_t ckA; // UBX mesaj doğrulama byte A
                uint8_t ckB; // UBX mesaj doğrulama byte B

                uint8_t payload[100]; // UBX mesaj yükü (maksimum 100 byte)

                GpsState gps; // GPS durumu

                void parseByte(uint8_t c); // Seri porttan gelen her byte'ı işleyen fonksiyon
                void processPacket(); // Tam bir UBX mesajı alındığında çağrılan fonksiyon

            public:
                GpsSensor();

                bool init();
                void update();

                bool hasFix();
                bool isHealthy() const;

                int32_t getLat();
                int32_t getLon();
                int32_t getAlt();

        };
    }
}