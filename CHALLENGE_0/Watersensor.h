#ifndef Watersensor_h
#define Watersensor_h
#include <Arduino.h>

class Watersensor{
  public:
    Watersensor(int _sensorPin,int _sensorPower);
    float readSensor();
  
  private:
    int _val;     //water level value
    int _sensorPower;   //Digital Output Pin
    int _sensorPin;   //Analog input pin

};

#endif
