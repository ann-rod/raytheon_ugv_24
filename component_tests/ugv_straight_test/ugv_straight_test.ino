#include "Motor.h"
//#include "Encoder.h"
#include <Servo.h>

//// MOTOR START
// motor pins
#define MOTOR_FL 8
#define MOTOR_FR 9
#define MOTOR_BL 10
#define MOTOR_BR 11

Motor mfr = Motor(MOTOR_FR, true);

Motor motors[4] = {Motor(MOTOR_FL, false),mfr,\ 
                   Motor(MOTOR_BL, false),Motor(MOTOR_BR, true)}; // array of all motors
//// MOTOR END


//// ENCODER START
#define ENCODER_FL 19
//#define ENCODER_FR 18
//#define ENCODER_BL 21
//#define ENCODER_BR 20

//Encoder enc_fl = Encoder(wheelDiameter, stepsPerRotation, units, ENCODER_FL);

struct speedCheckReturn{
  double currentSpeed;
  double distanceTraveled;
  unsigned long totalSteps;       // total steps triggered - important for persistent odometry
  unsigned long stepTimes[3];     // array for tracking step times - must be declared externally so it is persistent trigger to trigger
  unsigned int stepTimesIndex;        // tracks the index of the time tracking array for speed calculations
  unsigned int lastEncoderState;  // records previous encoder state
  unsigned int encoderPin;        // pin number for encoder
};

speedCheckReturn MOTOR_FL_SPEED;  // speedCheckReturn init
//speedCheckReturn MOTOR_FR_SPEED;
//speedCheckReturn MOTOR_BL_SPEED;
//speedCheckReturn MOTOR_BR_SPEED;

double wheelDiameter = 0.13;    // wheel diameter in meters
double stepsPerRotation = 20.0; // number of slots in encoder wheel
String units = "imperial";        // select "imperial", "metric", or "rotational"
unsigned int stepTimesLength = sizeof(MOTOR_FL_SPEED.stepTimes)/sizeof(MOTOR_FL_SPEED.stepTimes[0]); // length of stepTimes array
// IMPORTANT: stepTimesLength determines number of rotations speed will be averaged over as (stepTimesLength/stepsPerRotation)
//// ENCODER END



float totalDistTraveled = 0.0; // distance counter
float DISTANCE_TO_GO = 15.0; // in feet

// time vars
float prevTime; // in milliseconds
float deltaT;   // in milliseconds
float currTime; // IN SECONDS



void setup() {
  Serial.begin(9600);
  Serial.print("in setup\n");

  //// INITIALIZATIONS ////
  currTime = millis(); // init time
  init_motors();
  initEncoders();

  // motors test
  /*
  int power = 20;
  ugv_forward(power);
  delay(5000);
  ugv_backward(power);
  delay(5000);
  ugv_stop();
  Serial.print("end setup\n");*/

  

  //// ENCODER START
  

  //enc_fl.init_encoder();
  //attachInterrupt(digitalPinToInterrupt(ENCODER_FL), enc_fl.updateEncoder, CHANGE); // Call updateEncoder when any high/low changed seen
  //// ENCODER END*/
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("in loop");

  ugv_forward(35);
  //mfr.powerAdjust = 30;
  //goStraight(DISTANCE_TO_GO, totalDistTraveled, 20); // new var defined up top and set to 20


  //// ENCODER START
  //encoder = encoder.speedCheck(encoder);
  //// ENCODER END

  // updates
  //updateTime(); // update time vars and deltaT
  //totalDistTraveled += getAvgSpeed() * deltaT; // // update distance traveled
}

//// INIT. FUNCTIONS ////
void init_motors(){
  // init all motors in motor array
  for(int i=0;i<4;i++){
    motors[i].initialize();
  }
}



//// UGV BASIC CONTROLS////
void ugv_forward(int power){
  // move all motors forward at given power
  for(int i=0;i<4;i++){
    motors[i].forward(power);
  }
}

void ugv_backward(int power){
  // move all motors backward at a given power
  for(int i=0;i<4;i++){
    motors[i].backward(power);
  }
}

void ugv_stop(){
  // stop all ugv motors
  for(int i=0;i<4;i++){
    motors[i].stop();
  }
}

////////////////////
void updateTime(){
  prevTime = currTime; 
  currTime = millis();
  deltaT = (currTime - prevTime)/1000;
}

void goStraight(float dToGo, float dTraveled, float power){
  if(dTraveled < dToGo){
    //power = speed2power(speed);
    ugv_forward(power); // all motors assigns power PLUS the manual modifier for that wheel.
    //checkWheelsMatchSpeed(speed); // if check wheels is on and tries to correct speed it will change the modifier of the front wheels
  }
}

float getAvgSpeed(){
  float avg = MOTOR_FL_SPEED.currentSpeed;
  //avg += MOTOR_FR_SPEED.currentSpeed;
  //avg += MOTOR_BL_SPEED.currentSpeed;
  //avg += MOTOR_BR_SPEED.currentSpeed;
  return avg/1;
}




////////// ENCODER FUNCTIONS //////////

void initEncoders(){
  // Initialize the encoders. Set struct pins, set pin modes,
  // set initial speeds, attach interrupts.

  // Rotary Encoder Inputs - must be 18, 19, 20, or 21 on MEGA for ISR enabled pins  
  MOTOR_FL_SPEED.encoderPin = ENCODER_FL;
  //MOTOR_FR_SPEED.encoderPin = ENCODER_FR;
  //MOTOR_BL_SPEED.encoderPin = ENCODER_BL;
  //MOTOR_BR_SPEED.encoderPin = ENCODER_BR;

  // Set encoder pins as inputs
  pinMode(MOTOR_FL_SPEED.encoderPin, INPUT);
  //pinMode(MOTOR_FR_SPEED.encoderPin, INPUT);
  //pinMode(MOTOR_BL_SPEED.encoderPin, INPUT);
  //pinMode(MOTOR_BR_SPEED.encoderPin, INPUT);

  // Read the initial state of encoder1
  MOTOR_FL_SPEED.lastEncoderState = digitalRead(MOTOR_FL_SPEED.encoderPin);
  //MOTOR_FR_SPEED.lastEncoderState = digitalRead(MOTOR_FR_SPEED.encoderPin);
  //MOTOR_BL_SPEED.lastEncoderState = digitalRead(MOTOR_BL_SPEED.encoderPin);
  //MOTOR_BR_SPEED.lastEncoderState = digitalRead(MOTOR_BR_SPEED.encoderPin);

  // Call updateEncoder when any high/low changed seen
  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_SPEED.encoderPin), updateEncoderFL, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(MOTOR_FR_SPEED.encoderPin), updateEncoderFR, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(MOTOR_FR_SPEED.encoderPin), updateEncoderBL, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(MOTOR_FR_SPEED.encoderPin), updateEncoderBR, CHANGE);
}

void updateEncoderFL(){
  // call helper to update global vars outside ISR
  MOTOR_FL_SPEED = updateEncoderHelper(MOTOR_FL_SPEED);
}

/*
void updateEncoderFR(){
  // call helper to update global vars outside ISR
  MOTOR_FR_SPEED = updateEncoderHelper(MOTOR_FR_SPEED);  
}

void updateEncoderBL(){
  // call helper to update global vars outside ISR
  MOTOR_BL_SPEED = updateEncoderHelper(MOTOR_BL_SPEED);
}

void updateEncoderBR(){
  // call helper to update global vars outside ISR
  MOTOR_BR_SPEED = updateEncoderHelper(MOTOR_BR_SPEED);
}*/

speedCheckReturn updateEncoderHelper(speedCheckReturn UE){  // update global vars
  int currentEncoderState = digitalRead(UE.encoderPin);   // Read the current state
  ////
  if (currentEncoderState != UE.lastEncoderState  && currentEncoderState == 1){    // If last and current state are different, then pulse occurred
      UE.totalSteps++;
      if (UE.stepTimesIndex == (stepTimesLength-1)){     // if at end of array, cycles back to pos 0 - FIFO array
        UE.stepTimesIndex = 0;}
      else{
        UE.stepTimesIndex++;}
      UE.stepTimes[UE.stepTimesIndex] = millis();
      }
  UE.lastEncoderState = currentEncoderState;  // Remember last encoder state
  return UE;
}

speedCheckReturn speedCheck(speedCheckReturn SCR){
  unsigned long deltaT;   
  double currentRPS;      // current rotations per second
  double totalRotations;  // finds the number of rotations completed based on the number of encoder state changes that should occur per rotation
  double travelPerRotation = wheelDiameter*PI;  // distance traveled per wheel rotation
  ////
  totalRotations = SCR.totalSteps/stepsPerRotation;
  ////
  if (SCR.stepTimesIndex == (stepTimesLength-1)){         // finds time difference over one "revolution" of stepTimes
    deltaT = SCR.stepTimes[SCR.stepTimesIndex] - SCR.stepTimes[0];    
  } else {
    deltaT = SCR.stepTimes[SCR.stepTimesIndex] - SCR.stepTimes[SCR.stepTimesIndex+1];
  }
  currentRPS = (stepTimesLength/stepsPerRotation)/(deltaT/1000.0);
  ////
  if (units == "metric"){
    SCR.currentSpeed = currentRPS*travelPerRotation;      // speed in meters/second
    SCR.distanceTraveled = totalRotations*travelPerRotation;  // distance traveled in meters
  }
  if (units == "imperial"){
    SCR.currentSpeed = currentRPS*travelPerRotation*2.23694;      // speed in miles per hour
    SCR.distanceTraveled = totalRotations*travelPerRotation*3.28084;  // distance traveled in feet
  }
  if (units == "rotational"){
    SCR.currentSpeed = currentRPS/60.0;       // speed in rotations per minute
    SCR.distanceTraveled = totalRotations;    // rotations completed
  }
  return SCR;
}
