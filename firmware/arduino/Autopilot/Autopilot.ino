#include "core/Autopilot.h"

atabey::core::Autopilot autopilot;

void setup() {
    autopilot.begin();
}

void loop() {
    autopilot.update();
}