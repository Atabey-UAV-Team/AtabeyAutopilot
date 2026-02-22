#pragma once
#include <Arduino.h>

#include "../drivers/sensors/ISensor.h"
#include "../drivers/actuators/IActuator.h"
#include "../estimation/IEstimator.h"
#include "../control/IController.h"
#include "../comm/ICommLink.h"

#include "Scheduler.h"
#include "FlightModeManager.h"
#include "FailsafeManager.h"
#include "HealthMonitor.h"
#include "ParameterStore.h"

namespace atabey::drivers {
    class ISensor;
    class IActuator;
}

namespace atabey::estimation {
    class IEstimator;
}

namespace atabey::control {
    class IController;
}

namespace atabey::comm {
    class ICommLink;
}

namespace atabey {
    namespace core {

        class Scheduler;
        class FlightModeManager;
        class FailsafeManager;
        class HealthMonitor;
        class ParameterStore;

        class Autopilot {
        private:
            // Zaman Yönetimi (Scheduler)
            uint32_t lastMs;
            float dt;

            // Durum Kestirimi (Estimator output)
            float roll;
            float pitch;
            float yaw;

            float rollRate;
            float pitchRate;
            float yawRate;

            // Hedefler
            float desiredRoll;
            float desiredPitch;
            float desiredYaw;
            float desiredThrottle;

            // Aktüatör Çıkışları
            float aileron;
            float elevator;
            float rudder;
            float throttle;

            // Modüller
            atabey::drivers::ISensor* imu;
            atabey::drivers::ISensor* gps;
            atabey::estimation::IEstimator* estimator;
            atabey::control::IController* controller;
            atabey::drivers::IActuator* actuators;
            atabey::comm::ICommLink* commLink;

            Scheduler* scheduler;
            FlightModeManager* flightModeMgr;
            FailsafeManager* failsafeMgr;
            HealthMonitor* healthMonitor;
            ParameterStore* paramStore;

            // Core Adımlar
            void updateTime();
            void readSensors();
            void estimateState();
            void runControl();
            void applyActuators();
            void sendTelemetry();
            void checkFailsafe();

        public:
            Autopilot();

            // Bağımlılık Bağlama
            void attachIMU(atabey::drivers::ISensor* imuSensor);
            void attachGPS(atabey::drivers::ISensor* gpsSensor);
            void attachEstimator(atabey::estimation::IEstimator* est);
            void attachController(atabey::control::IController* ctrl);
            void attachActuators(atabey::drivers::IActuator* act);
            void attachComm(atabey::comm::ICommLink* comm);

            void attachScheduler(Scheduler* s);
            void attachFlightModeManager(FlightModeManager* fmm);
            void attachFailsafeManager(FailsafeManager* fm);
            void attachHealthMonitor(HealthMonitor* hm);
            void attachParameterStore(ParameterStore* ps);

            bool begin();
            void update();

            // Hedef belirleme (Manuel de kullanılabilir)
            void setTargets(float roll, float pitch, float yaw, float throttle);

            // Debug / getter fonksiyonları
            float getRoll() const;
            float getPitch() const;
            float getYaw() const;
        };

    }
}