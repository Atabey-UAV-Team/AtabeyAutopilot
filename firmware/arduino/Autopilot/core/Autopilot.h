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

namespace atabey {
    namespace core {

        // Forward declarations (interface'ler başka modüllerde olacak)
        class ISensor;
        class IEstimator;
        class IController;
        class IActuator;
        class ICommLink;

        class Scheduler;
        class FlightModeManager;
        class FailsafeManager;
        class HealthMonitor;
        class ParameterStore;

        enum class FlightMode {
            MANUAL,
            STABILIZED,
            AUTO,
            RTL
        };

        enum class FailsafeReason {
            RC_LOST,
            GPS_LOST,
            LOW_BATTERY,
            SENSOR_FAILURE
        };

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
            ISensor* imu;
            ISensor* gps;
            IEstimator* estimator;
            IController* controller;
            IActuator* actuators;
            ICommLink* commLink;

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
            void attachIMU(ISensor* imuSensor);
            void attachGPS(ISensor* gpsSensor);
            void attachEstimator(IEstimator* est);
            void attachController(IController* ctrl);
            void attachActuators(IActuator* act);
            void attachComm(ICommLink* comm);

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