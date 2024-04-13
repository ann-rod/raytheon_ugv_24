/*#ifndef Encoder_h
#define Encoder_h
#include "Arduino.h"
class Encoder{
  public:
    struct speedCheckReturn{
      double currentSpeed;
      double distanceTraveled;
      unsigned long totalSteps;      // total steps triggered - important for persistent odometry
      unsigned long stepTimes[3];    // array for tracking step times - must be declared externally so it is persistent trigger to trigger
      unsigned int stepTimesIndex;   // tracks the index of the time tracking array for speed calculations
      unsigned int lastEncoderState; // records previous encoder state
      unsigned int encoderPin;       // pin number for encoder
    };
    
    Encoder::Encoder(double wheelDiameter, double stepsPerRotation, String units, int pin);
    speedCheckReturn speedCheck(speedCheckReturn SCR);
    double getCurrentSpeed();
    double getDistanceTraveled();
    void init_encoder();
    void updateEncoder();

  private:
    double _wheelDiameter;
    double _stepsPerRotation;
    String _units;
    unsigned int _stepTimesLength;
    speedCheckReturn _motorSpeed; // speedCheckReturn init
    
    speedCheckReturn updateEncoderHelper(speedCheckReturn UE);
    
};
#endif*/