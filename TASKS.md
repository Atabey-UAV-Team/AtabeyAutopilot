# 🛠 ATABEY UAV – Geliştirme Görev Listesi

Bu doküman, **Atabey İHA Fixed-Wing Autopilot** projesinin **aktif görevlerini, sorumluları ve ilerleme durumunu** takip etmek için kullanılmaktadır.

Durumlar: **⬜ Yapılacak | 🟨 Devam Ediyor | ✅ Tamamlandı**

---

## 🔥 Öncelikli Görevler (Deadline - 15.02.2026)

- [ ] 🟨 IMU sürücüsünün temel okuma fonksiyonları (Hatice)
- [ ] 🟨 GPS kütüphanesi entegrasyonu (Şiar)
- [ ] 🟨 PID kontrolcü temel iskeletinin oluşturulması (Eray)
- [ ] 🟨 Autopilot ana kütüphanesinin temellerinin oluşturulması (Furkan)
- [ ] 🟨 Telemetri paket formatının belirlenmesi (Muhammet)
- [ ] 🟨 Sensör filtreleme için EKF taslak yapısı (Mert)

---

## 🧠 Yazılım Altyapısı

### Core
- [ ] 🟨 Zamanlayıcı (Scheduler) iskeleti  
- [ ] ⬜ Task kayıt / önceliklendirme yapısı  
- [ ] ⬜ Sistem durum makinesi (INIT, STANDBY, ARMED, FAILSAFE)

### Drivers
- [ ] 🟨 IMU (MPU6050/9250) temel sürücü  
- [ ] ⬜ Barometre sürücüsü  
- [ ] 🟨 GPS sürücüsü  

### Control
- [ ] 🟨 Roll/Pitch PID kontrolcü  
- [ ] ⬜ Servo miksleme  
- [ ] ⬜ Limit ve saturasyon katmanı  

### Comm
- [ ] ⬜ UART telemetri protokolü  
- [ ] 🟨 GCS mesaj formatı  
- [ ] ⬜ Komut alma (arm/disarm, mod değişimi)

---

## 🧪 Test & Doğrulama

- [ ] ⬜ Sensör verisi simülasyonu (fake data)  
- [ ] ⬜ HIL için test altyapısı  
- [ ] ⬜ MATLAB/Simulink sonuçlarının embedded çıktılarla karşılaştırılması  

---

## 📅 Uzun Vadeli Hedefler

- [ ] ⬜ Kumandayla manuel uçuş 
- [ ] ⬜ EKF tabanlı durum kestirimi  
- [ ] ⬜ Otonom görev yürütme  
- [ ] ⬜ Fail-safe senaryoları (GPS kaybı, RC kaybı, düşük batarya)  
- [ ] ⬜ MAVLink uyumluluğu
