#include "GpsDriver.h"

namespace atabey {
  namespace drivers{
   
    bool GpsDriver::init(){
      bool buffer; //state control
      Serial1.begin(9600); //start communication with gps
      if(Serial1.avaliable() != 0){
        buffer = true; //if gps is avaliable, set state 1
        digitalWrite(31, 1); //if gps is working, blink LED connected to digital 31
      }else() buffer = false; // if gps isn't working, set state 0
      return buffer; // return stat
    }

    void GpsDriver::update(){
      char data, arr[75], oldData = '\0';
      int8_t remainder = 0, index = -1, int count = 0;
      char data;
      uint32_t north, northDegree, northMinute, east, eastDegree, eastMinute;
      float northAngle, altitude = 0.0, decimalMultiplier = 0.1;
      bool isDecimal = false, isNegative = false;
      
      if(Serial1.available()){ 
        data = Serial1.read();
        
        if(data == 'A' && oldData == 'G'){
          remainder = 74; 
        }else{
          if(remainder > 0){
            arr[74 - remainder] = data; 
            remainder--; 
          }
          else if(remainder == 0){
            for(int i=11; i<=21; i++) arr[i] -= 48; 
            for(int i=24; i<=34; i++) arr[i] -= 48; 
            
            northDegree = 10 * arr[11] + arr[12];
            northMinute = 1000000 * arr[13] + 100000 * arr[14] + 10000 * arr[16] + 1000 * arr[17] + 100 * arr[18] + 10 * arr[19] + arr[20];
            north = 6000000 * northDegree + northMinute;
          
            northAngle = (float)northDegree + (((float)northMinute)/6000000.0) * 0.01745329251994329576923690768489;
            eastDegree = 100 * arr[24] + 10 * arr[25] + arr[26];
            eastMinute = 1000000 * arr[27] + 100000 * arr[28] + 10000 * arr[30] + 1000 * arr[31] + 100 * arr[32] + 10 * arr[33] + arr[34];
            east = (uint32_t)(((float)6000000 * eastDegree + eastMinute) * cos(northAngle) * 1.852);
            
            for(int i = 0; i < 74; i++) {
                if(arr[i] == ',') {
                    count++;
                    if(count == 9) { 
                        index = i + 1;
                        break;
                    }
                }
            }
            
            if (index != -1) {
                if (arr[index] == '-') {
                    isNegative = true;
                    index++;
                }
                for(int i = index; i < 74; i++) {
                    if(arr[i] == ',') break; 
                    if(arr[i] == '.') {
                        isDecimal = true;
                    } 
                    else if(arr[i] >= '0' && arr[i] <= '9') {
                        if(!isDecimal) {
                            altitude = (altitude * 10.0) + (arr[i] - 48);
                        } else {
                            altitude = altitude + ((arr[i] - 48) * decimalMultiplier);
                            decimalMultiplier /= 10.0;
                        }
                    }
                }
                if (isNegative) altitude = -altitude;
            }
            lat = north;
            lon = east;
            alt = altitude;
            remainder--;
          }
        }
        oldData = data;
      }
    }

    bool GpsDriver::isHealthy(){
      while(Serial1.available()){
        unsigned long lastUpdate;
        char identificationData = '\0';
        bool state = false;
        while(state == false){
          identificationData = Serial1.read();
          if (Serial1.available() != 0 && identificationData != '\0'){
            lastUpdate = millis();
            state = true;
        }
        identificationData = '\0';
        if(millis() - lastUpdate > 2000){
          return false;
        }else{
          return true;
        }
      }
    }

    bool GpsDriver::hasFix(){
      return(lat != 0 && lon != 0);
    }

    double GpsDriver::getLat(){
      return lat;
    }

    double GpsDriver::getLon(){
      return lon;
    }

    float GpsDriver::getAlt(){
      return alt;
    }
  }
}
