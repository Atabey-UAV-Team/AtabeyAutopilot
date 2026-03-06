#ifndef GPSDRIVER_H
#define GPSDRIVER_H
#include <Arduino.h>

class GpsDriver{

private:
    double lat;
    double lon;
    double alt;

public:
    bool init();
    void update();
    bool isHealthy();
    double getLat();
    double getLon();
    float getAlt();
    bool hasFix();

};

#endif