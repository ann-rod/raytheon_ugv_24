/*#include "Arduino.h"
#include "Encoder.h"
/*
struct Encoder::speedCheckReturn{
  double currentSpeed;
  double distanceTraveled;
  unsigned long totalSteps;      // total steps triggered - important for persistent odometry
  unsigned long stepTimes[3];    // array for tracking step times - must be declared externally so it is persistent trigger to trigger
  unsigned int stepTimesIndex;   // tracks the index of the time tracking array for speed calculations
  unsigned int lastEncoderState; // records previous encoder state
  unsigned int encoderPin;       // pin number for encoder
};

double _wheelDiameter;
double _stepsPerRotation;
String _units;
unsigned int _stepTimesLength;
struct Encoder::speedCheckReturn _motorSpeed; // speedCheckReturn init

Encoder::Encoder(double wheelDiameter, double stepsPerRotation, String units, int pin){
  // initialize Encoder object
  _wheelDiameter = wheelDiameter;
  _stepsPerRotation = stepsPerRotation;
  _units = units;
  _stepTimesLength = sizeof(_motorSpeed.stepTimes)/sizeof(_motorSpeed.stepTimes[0]); // length of stepTimes array
  // IMPORTANT: stepTimesLength determines number of rotations speed will be averaged over as (stepTimesLength/stepsPerRotation)
  _motorSpeed.encoderPin = pin;
}

Encoder::speedCheckReturn Encoder::speedCheck(speedCheckReturn SCR){
  unsigned long deltaT;   
  double currentRPS;      // current rotations per second
  double totalRotations;  // finds the number of rotations completed based on the number of encoder state changes that should occur per rotation
  double travelPerRotation = _wheelDiameter*PI;  // distance traveled per wheel rotation
  
  totalRotations = SCR.totalSteps/_stepsPerRotation;
  
  if (SCR.stepTimesIndex == (_stepTimesLength-1)){         // finds time difference over one "revolution" of stepTimes
    deltaT = SCR.stepTimes[SCR.stepTimesIndex] - SCR.stepTimes[0];    
  } else {
    deltaT = SCR.stepTimes[SCR.stepTimesIndex] - SCR.stepTimes[SCR.stepTimesIndex+1];
  }
  currentRPS = (_stepTimesLength/_stepsPerRotation)/(deltaT/1000.0);

  if (_units == "metric"){
    SCR.currentSpeed = currentRPS*travelPerRotation;        // speed in meters/second
    SCR.distanceTraveled = totalRotations*travelPerRotation;// distance traveled in meters
  }
  if (_units == "imperial"){
    SCR.currentSpeed = currentRPS*travelPerRotation*2.23694;        // speed in miles per hour
    SCR.distanceTraveled = totalRotations*travelPerRotation*3.28084;// distance traveled in feet
  }
  if (_units == "rotational"){
    SCR.currentSpeed = currentRPS/60.0;       // speed in rotations per minute
    SCR.distanceTraveled = totalRotations;    // rotations completed
  }
  return SCR;
}

double Encoder::getCurrentSpeed(){
  _motorSpeed = speedCheck(_motorSpeed);
  return _motorSpeed.currentSpeed;
}

double Encoder::getDistanceTraveled(){
  return _motorSpeed.distanceTraveled;
}

Encoder::speedCheckReturn Encoder::updateEncoderHelper(speedCheckReturn UE){
  // update global vars
  int currentEncoderState = digitalRead(UE.encoderPin); // Read the current state
  
  if (currentEncoderState != UE.lastEncoderState  && currentEncoderState == 1){
    // If last and current state are different, then pulse occurred
      UE.totalSteps++;
      if (UE.stepTimesIndex == (_stepTimesLength-1)){    // if at end of array, cycles back to pos 0 - FIFO array
        UE.stepTimesIndex = 0;}
      else{
        UE.stepTimesIndex++;}
      UE.stepTimes[UE.stepTimesIndex] = millis();
      }
  UE.lastEncoderState = currentEncoderState;            // Remember last encoder state
  return UE;
}

void Encoder::updateEncoder(){
  // call helper to update global vars outside ISR
  _motorSpeed = updateEncoderHelper(_motorSpeed);
}


void Encoder::init_encoder(){
  pinMode(_motorSpeed.encoderPin, INPUT); //Set encoder pins as inputs
  _motorSpeed.lastEncoderState = digitalRead(_motorSpeed.encoderPin); // Read the initial state of encoder1
  attachInterrupt(digitalPinToInterrupt(_motorSpeed.encoderPin), Encoder::updateEncoder, CHANGE); // Call updateEncoder when any high/low changed seen
}*/