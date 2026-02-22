# ✈ ATABEY UAV Fixed-Wing Autopilot

Bu repo, **Atabey İHA** projesi kapsamında geliştirilen, **Arduino tabanlı sabit kanat (fixed-wing) otonom uçuş kontrol yazılımını** içermektedir. Sistem; sensör okuma, durum kestirimi, kontrol algoritmaları, görev yürütme ve yer istasyonu haberleşmesini **modüler ve genişletilebilir** bir mimariyle ele alır.

---

## 🎯 Proje Hedefleri

- Sabit kanat İHA için **tam otonom uçuş yeteneği**
- Arduino üzerinde **gerçek zamanlı kontrol**
- **Modüler mimari** (sürücüler, kontrol, haberleşme, görev mantığı)
- MATLAB/Simulink ile **sistem modelleme ve doğrulama**
- Yer istasyonu (GCS) ile **telemetri ve görev entegrasyonu**

---

## 🧠 Sistem Mimarisi (Özet)

Autopilot yazılımı aşağıdaki ana katmanlardan oluşur:

- **Drivers**  
  Donanım bağımlı katman (IMU, barometre, GPS, UART, I2C, SPI vb.)

- **Core**  
  Zamanlayıcı (Scheduler), görev döngüsü ve sistem altyapısı

- **Control**  
  PID tabanlı kontrol algoritmaları (Roll, Pitch, Yaw, hız vb.)

- **Comm**  
  Yer istasyonu / telemetri haberleşmesi (MAVLink benzeri yapı)

- **Estimation**  
  Sensör verilerinden uçağın durumunu (roll, pitch, yaw, hız, konum vb.) kestiren katman

---

## 📁 Dizin Yapısı
```
AtabeyAutopilot/
│
├─ src/ # Arduino library (AtabeyAutopilot)
│ ├─ AtabeyAutopilot.h # Umbrella header
│ ├─ core/ # Autopilot core, scheduler, flight modes, failsafe
│ ├─ control/ # Kontrol katmanı (PID, controller arayüzleri)
│ ├─ estimation/ # Durum kestirimi (IEstimator, EKF vb.)
│ ├─ drivers/ # Sensör ve aktüatör sürücüleri (IMU, GPS, Servo)
│ └─ comm/ # Telemetri ve haberleşme katmanı (MAVLink, LoRa)
│
├─ test/ # Arduino test sketch’leri ve donanım doğrulama kodları
├─ diagrams/ # UML class/sequence diyagramları
├─ MATLAB/ # Simülasyon, modelleme ve kontrol analizleri
│ ├─ scripts/
│ ├─ simulinkModelleri/
│ └─ sistemDinamikleri/
│
├─ ARCHITECTURE.md # Yazılım mimarisi ve katmanlar
├─ TASKS.md # Yol haritası ve görev takibi
├─ README.md # Proje genel açıklaması
├─ LICENSE
└─ library.properties # Arduino library metadata
```

---

## 📐 Kodlama Standartları

- Tüm modüller arayüz (interface) üzerinden core’a bağlanır.
- Donanıma bağımlı kodlar yalnızca `drivers/` altında tutulur.
- Core katmanı donanımdan tamamen bağımsızdır.
- Zamanlama Scheduler üzerinden yönetilir (delay kullanılmaz).

---

## 🔁 Çalışma Mantığı (Yüksek Seviye)

1. **MCU Boot**
2. Sensör ve haberleşme birimlerinin başlatılması
3. Scheduler üzerinden periyodik görevlerin çalıştırılması
4. Sensör verilerinin okunması
5. Durum kestirimi (attitude, hız, irtifa)
6. Kontrol algoritmalarının çalıştırılması
7. Aktüatör komutlarının üretilmesi
8. Telemetri verilerinin GCS’ye gönderilmesi

---

## 📊 MATLAB & Simulink

Bu repo, gömülü yazılım ile **aynı sistemin matematiksel modelini** de içerir:

- Uçak dinamikleri
- PID kontrolcü tasarımı
- Simulink tabanlı sistem doğrulama
- Uçuş senaryosu simülasyonları

Bu sayede:
- Gerçek uçuş öncesi kontrolcü ayarları test edilir.
- Embedded kod ile teorik model arasında tutarlılık sağlanır.

---

## 🛠 Donanım Hedefi

- **MCU:** Arduino Mega2560 Pro Mini
- **Sensörler:** IMU (MPU6050/9250), Barometre, GPS
- **Haberleşme:** UART / LoRa / RC
- **Aktüatörler:** Servo yüzeyler + ESC

---

## 🚀 Geliştirme Notları

- Kod yapısı **Arduino** mimarisine yöneliktir.
- Her alt sistem bağımsız geliştirilebilir.
- Scheduler tabanlı yapı dolayısıyla sonradan RTOS’a geçişe uygundur.
- PID ve kontrol katmanı kolayca genişletilebilir.

---

## 📌 Yol Haritası (Özet)

- [x] Mimari Planlaması
- [ ] Sensör Driverleri / %60
- [ ] Durum kestirimi (EKF) / %0
- [ ] Uçuş modları (AUTO, MANUAL) / %10
- [ ] MAVLink uyumluluğu / Planlandı
- [ ] Donanım-in-the-loop (HIL) testleri / Başlanmadı
- [ ] Fail-safe ve güvenlik katmanları / Başlanmadı

---

## 👥 Ekip ve Görev Dağılımı

Bu proje **Atabey İHA Elektronik Birimi** tarafından geliştirilmektedir. Otopilot yazılımı, modüler bir mimari ile ekip üyeleri arasında paylaştırılmıştır.

- **Furkan** — Ana yazılım mimarisi, sürücüler, sınıf diyagramları ve modüller arası entegrasyon
- **Eray** — Uçuş kontrol algoritmaları (PID, stabilizasyon, kontrol döngüleri)
- **Şiar** — GPS sürücüsü geliştirme, navigasyon ve kaçış (failsafe / recovery) algoritmaları
- **Mert** — Sensör verisi filtreleme (ör. EKF), durum kestirimi (state estimation)
- **Hatice** — IMU sürücüsü geliştirme ve PCB tasarımı
- **Jad** — Uçuş kontrol algoritmaları (PID, stabilizasyon, kontrol döngüleri)

Bu yapı sayesinde yazılım; **sürücüler, kontrol, navigasyon ve haberleşme** katmanları arasında net bir şekilde ayrılmıştır.

---
