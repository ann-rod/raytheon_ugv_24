#include "Motor.h"
#include <Servo.h>
#include <TimerOne.h>

//// MOTOR START
// motor pins
#define MOTOR_FL 8
#define MOTOR_FR 9
#define MOTOR_BL 10
#define MOTOR_BR 11

Motor motors[4] = {Motor(MOTOR_FL, false),Motor(MOTOR_FR, true),\ 
                   Motor(MOTOR_BL, false),Motor(MOTOR_BR, true)}; // array of all motors
//// MOTOR END


//// ENCODER START ////
#define ENCODER_FR 19 // 18 is FL, 19 is FR
double wheelDiameter = 0.13; // wheel diam. in meters
double stepsPerRotation = 20.0; // num. slots in encoder wheel
float conversionFactor = 0.142169*PI; // convert rot/s to yard/s
//float conversionFactor = 0.290802*PI; //convert rot/s to mph
unsigned int enc_counter=0; // counter for FL encoder
float enc_speed; // FL encoder speed in rotations/sec

void doCountEnc(){ 
  /*
  Counter for Encoder
  */
  enc_counter++;
  //Serial.print("on\n");
} 

void timerIsr(){
  /* 
  Interrupt Function for Encoder
  */
  //Serial.print("in timerisr\n");
  Timer1.detachInterrupt();  //stop the timer
  enc_speed = (enc_counter / stepsPerRotation) *conversionFactor;  // divide by number of holes in Disc, mult. by conversion Factor
  enc_counter=0;  //  reset counter to zero
  Timer1.attachInterrupt(timerIsr);  //enable the timer
}
///// ENCODER END /////

void setup() {
  Serial.begin(9600);

  //// INITIALIZATIONS ////
  init_motors();
  init_encoder();
}

void loop() {
  ugv_forward(20);

  Serial.print("Motor Speed: ");
  Serial.print(enc_speed,DEC);  
  Serial.println(" yards per second"); 
}


//// INIT. FUNCTIONS ////
void init_motors(){
  // init all motors in motor array
  for(int i=0;i<4;i++){
    motors[i].initialize();
  }
}

void init_encoder(){
  /*
  Initialize Encoder
  */
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt(ENCODER_FR), doCountEnc, HIGH);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt(timerIsr); // enable the timer
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

