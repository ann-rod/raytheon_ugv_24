#include <Arduino.h>
#include "Watersensor.h"

    int _val;     //water level value
    int _sensorPower;   //Digital Output Pin
    int _sensorPin;   

Watersensor::Watersensor(int sensorPin,int _sensorPower){
  pinMode(_sensorPower, OUTPUT);     // Set sensorPower as OUTPUT pin
  digitalWrite(_sensorPower, LOW);   // Set to LOW so no power flows through the sensor
  int _val = 0;     // Value for storing water level
}

float Watersensor::readSensor() {
	digitalWrite(_sensorPower, HIGH);	// Turn the sensor ON
	delay(10);							// wait 10 milliseconds
	_val = analogRead(_sensorPin);		// Read the analog value form sensor
	digitalWrite(_sensorPower, LOW);		// Turn the sensor OFF
	return _val;							// send current reading
}


