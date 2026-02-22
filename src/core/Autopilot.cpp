#include "Autopilot.h"

#include "../drivers/sensors/ISensor.h"
#include "../estimation/IEstimator.h"
#include "../control/IController.h"
#include "../drivers/actuators/IActuator.h"
#include "../comm/ICommLink.h"

namespace atabey {
    namespace core {
    
        Autopilot::Autopilot()
            : lastMs(0), dt(0.0f),
            roll(0), pitch(0), yaw(0),
            rollRate(0), pitchRate(0), yawRate(0),
            aileron(0), elevator(0), rudder(0), throttle(0),
            desiredRoll(0), desiredPitch(0), desiredYaw(0),
            controller(nullptr), estimator(nullptr), commLink(nullptr),
            imu(nullptr), gps(nullptr), actuators(nullptr),
            scheduler(nullptr), flightModeMgr(nullptr),
            failsafeMgr(nullptr), healthMonitor(nullptr), paramStore(nullptr)
        {}

        // Modüller
        void Autopilot::attachIMU(atabey::drivers::ISensor* imuSensor) { imu = imuSensor; }
        void Autopilot::attachGPS(atabey::drivers::ISensor* gpsSensor) { gps = gpsSensor; }
        void Autopilot::attachEstimator(atabey::estimation::IEstimator* est) { estimator = est; }
        void Autopilot::attachController(atabey::control::IController* ctrl) { controller = ctrl; }
        void Autopilot::attachActuators(atabey::drivers::IActuator* act) { actuators = act; }
        void Autopilot::attachComm(atabey::comm::ICommLink* comm) { commLink = comm; }

        void Autopilot::attachScheduler(Scheduler* s) { scheduler = s; }
        void Autopilot::attachFlightModeManager(FlightModeManager* fmm) { flightModeMgr = fmm; }
        void Autopilot::attachFailsafeManager(FailsafeManager* fm) { failsafeMgr = fm; }
        void Autopilot::attachHealthMonitor(HealthMonitor* hm) { healthMonitor = hm; }
        void Autopilot::attachParameterStore(ParameterStore* ps) { paramStore = ps; }

        // Lifecycle
        bool Autopilot::begin() {
            lastMs = millis();

            bool ok = true;
            if (imu) ok &= imu->init();
            if (gps) ok &= gps->init();
            if (estimator) ok &= estimator->init();
            if (controller) ok &= controller->init();
            if (actuators) ok &= actuators->init();
            if (commLink) ok &= commLink->init();

            return ok;
        }

        void Autopilot::update() {
            updateTime();
            readSensors();
            estimateState();
            runControl();
            applyActuators();
            sendTelemetry();
            checkFailsafe();
        }

        // Ana adımlar
        void Autopilot::updateTime() {
            uint32_t now = millis();
            dt = (now - lastMs) * 0.001f;
            lastMs = now;

            // İlk çalışmadaki lastMs = 0 durumunda dt = 0 gelme durumunun engellenmesi durumu,
            // ve uint32_t limiti aşıldığında oluşan overflow sonucu dt negatif gelmesi engellenmek için bu "if" bloğu kullanılmıştır.
            if (dt <= 0.0f || dt > 0.1f) {
                dt = 0.01f; // 10ms değeri tamamen random
            }
        }

        void Autopilot::readSensors() {
            if (imu) imu->update();
            if (gps) gps->update();
        }

        void Autopilot::estimateState() {
            if (!estimator) return;

            estimator->update(dt);

            roll = estimator->getRoll();
            pitch = estimator->getPitch();
            yaw = estimator->getYaw();

            rollRate = estimator->getRollRate();
            pitchRate = estimator->getPitchRate();
            yawRate = estimator->getYawRate();
        }

        void Autopilot::runControl() {
            if (!controller) return;

            controller->setTarget(desiredRoll, desiredPitch, desiredYaw);
            controller->update(dt);

            aileron = controller->getAileron();
            elevator = controller->getElevator();
            rudder = controller->getRudder();
            throttle = controller->getThrottle();
        }

        void Autopilot::applyActuators() {
            if (!actuators) return;

            actuators->setAileron(aileron);
            actuators->setElevator(elevator);
            actuators->setRudder(rudder);
            actuators->setThrottle(throttle);
        }

        void Autopilot::sendTelemetry() {
            if (!commLink) return;

            uint8_t buf[12]; // TODO: Telemetri Paketinin byte büyüklüğüne göre düzenlenecek
            
            commLink->send(buf, sizeof(buf)); // TODO
        }

        void Autopilot::checkFailsafe() {
            if (imu && !imu->isHealthy()) {
                // TODO: FailsafeManager entegre edilecek
            }

            if (gps && !gps->isHealthy()) {
                // TODO: RTL mode tetiklenebilir
            }
        }

        // TODO: Hedefler (Kontrol sistemi ile belirlenedek)
        void Autopilot::setTargets(float roll, float pitch, float yaw, float throttle) {
            desiredRoll     = roll;
            desiredPitch    = pitch;
            desiredYaw      = yaw;
            desiredThrottle = throttle;
        }

        float Autopilot::getRoll() const { return roll; }
        float Autopilot::getPitch() const { return pitch; }
        float Autopilot::getYaw() const { return yaw; }

    }
}