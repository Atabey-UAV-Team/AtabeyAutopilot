# ⚙ ATABEY Autopilot Namespace-Class Listesi

Bu doküman, **Atabey İHA Otopilot yazılımının modül yapısını, namespace organizasyonunu ve temel sınıflarını** tanımlar.
Amaç; ekip içinde **isimlendirme karmaşasını önlemek**, mimariyi baştan kilitlemek ve UML diyagramları için **tek referans noktası** oluşturmaktır.

Yeni sınıf veya modül eklenirken bu liste referans alınmalıdır.

---

## 📜 Namespace Organizasyonu

Ana kök namespace:

```text
atabey
```

Alt modüller:

```text
atabey::core
atabey::drivers
atabey::control
atabey::comm
atabey::estimation
atabey::utils        (opsiyonel)
```

> Not: Namespace yapısı, `src/` klasör yapısı ile birebir eşleşmelidir.

---

# 📦 atabey::core

**Sistem orkestrasyonu & yaşam döngüsü (lifecycle)**

Bu katman, tüm sistemi bir arada tutan çekirdektir.
Zamanlama, uçuş modu yönetimi, failsafe, parametreler ve alt modüllerin koordinasyonu buradan yürütülür.

### Ana Sınıflar

* `Autopilot`
* `Scheduler`
* `SystemState`
* `HealthMonitor`
* `ParameterStore`
* `FlightModeManager`
* `FailsafeManager`

### Core Tarafından Kullanılan Arayüzler (Dependency Inversion)

Core katmanı, alt modüllere **doğrudan somut sınıflar üzerinden değil, arayüzler üzerinden** bağlanır:

* `drivers::ISensor`
* `drivers::IActuator`
* `estimation::IEstimator`
* `control::IController`
* `comm::ICommLink`

> Not: Core katmanı donanımı ve protokol detaylarını **bilmemelidir**.
> Tüm bağımlılıklar interface üzerinden enjekte edilir (dependency injection).

---

# 📦 atabey::drivers

**Donanım soyutlama katmanı (HAL benzeri yapı)**

Bu katman donanıma bağımlıdır. Sensör, aktüatör ve çevre birimlerinin gerçek sürücüleri burada yer alır.

### Sensörler

* `ISensor`
* `ImuDriver`
* `GpsDriver`
* `BarometerDriver`
* `MagnetometerDriver`
* `AirspeedSensor`

### Aktüatörler

* `IActuator`
* `ServoDriver`
* `MotorDriver`

### Diğer Donanım Bileşenleri

* `FlashStorage`
* `RcReceiver`
* `PowerMonitor`

> Not: Üst katmanlar (core, control, estimation) donanımın **nasıl** çalıştığını bilmez; sadece arayüzü bilir.

---

# 📦 atabey::control

**Kontrol algoritmaları**

Bu katman, uçuş kontrolünün matematiksel ve algoritmik kısmını içerir.

* `IController`
* `PID`
* `RateController`
* `AttitudeController`
* `PositionController`
* `AltitudeController`
* `NavigationController`
* `TakeoffController`
* `LandingController`

> Not: Control katmanı doğrudan `drivers` ile konuşmaz.
> Gerekli durum bilgilerini `core` ve `estimation` üzerinden alır.

---

# 📦 atabey::estimation

**Durum kestirimi (state estimation)**

Sensör verilerinden uçağın gerçek durumunu tahmin eden katmandır.

* `IEstimator`
* `AttitudeEstimator`
* `PositionEstimator`
* `EkfEstimator`

> Not: Bu katman yalnızca `drivers::ISensor` arayüzlerini kullanır.
> Sensörün hangi marka/model olduğu estimation katmanını ilgilendirmez.

---

# 📦 atabey::comm

**Haberleşme & protokoller**

Yer istasyonu, telemetri ve komut alışverişi bu katmanda yapılır.

* `ICommLink`
* `TelemetryLink`
* `MavlinkLink`
* `LoraLink`
* `GroundStationInterface`
* `CommandParser`
* `LogStream`

> Not: Haberleşme protokolü değiştiğinde (`MAVLink → custom protocol` gibi),
> sadece `comm` katmanı etkilenmelidir. Core bundan bağımsız kalmalıdır.

---

# 📦 atabey::utils

**Ortak yardımcı yapılar**

Bu katman, tüm sistem tarafından kullanılabilecek genel araçları içerir.

* `MathUtils`
* `DspUtils`

> Not: `utils` hiçbir modüle bağımlı olmamalıdır.
> Bağımlılık yönü her zaman `utils → diğer modüller` şeklinde tek yönlüdür.

---

## 🔒 Mimari Kurallar (Kırmızı Çizgiler)

* `core`, `drivers` içindeki somut sınıfları doğrudan kullanmaz.
* Tüm modüller mümkün olduğunca **interface üzerinden** bağlanır.
* Donanıma bağımlı hiçbir kod `core`, `control` veya `estimation` içine sızmaz.
* Namespace yapısı ile klasör yapısı **her zaman birebir** kalır.

---

## 📐 UML İçin Referans

Bu doküman, oluşturulacak tüm UML class diagram ve sequence diagramları için
**tek referans mimari tanımıdır.**

UML diyagramları bu yapıdan sapamaz; mimari değişirse bu dosyanın güncellenmesi beklenir.

---