#include "Motor.h"
#include <Servo.h>
#include <TimerOne.h>
#include "Watersensor.h"
#include <Wire.h>
#include "Gyroscope.h"

//// MOTOR START
// motor pins
#define MOTOR_FL 8
#define MOTOR_FR 9
#define MOTOR_BL 10
#define MOTOR_BR 11

Motor motors[4] = {Motor(MOTOR_FL, false),Motor(MOTOR_FR, true),\ 
                   Motor(MOTOR_BL, false),Motor(MOTOR_BR, true)}; // array of all motors
//// MOTOR END


//// ENCODER START
#define ENCODER_FR 19 // FR =19, FL = 18
double wheelDiameter = 0.128; // wheel diam. in meters
double stepsPerRotation = 20.0; // num. slots in encoder wheel
float conversionFactor = wheelDiameter*1.09361*PI; // convert rot/s to yard/s
//float conversionFactor = wheelDiameter*1.15694*PI; //convert rot/s to mph
unsigned int enc_counter=0; // counter for FL encoder
float enc_speed; // FL encoder speed in rotations/sec

void doCountEnc(){ 
  /*
  Counter for Encoder
  */
  enc_counter++;
} 

void timerIsr(){
  /* 
  Interrupt Function for Encoder
  */
  Timer1.detachInterrupt();  //stop the timer
  enc_speed = (enc_counter / stepsPerRotation) * conversionFactor/4;  // divide by number of holes in Disc, mult. by conversion Factor
  enc_counter=0;  //  reset counter to zero
  Timer1.attachInterrupt(timerIsr);  //enable the timer
  //totalDistTraveled += enc_speed * deltaT; //update distance traveled
}
///// ENCODER END /////


//// WATERSENSOR START
  // Sensor pins
#define sensorPower 7 
#define sensorPin A0

Watersensor ws1 = Watersensor(sensorPin,7); //name water sensor  
//// WATERSENSOR END

//// GYROSCOPE START
/*
Gyroscope gyro;
float gyro_forward_pitch;
float curr_pitch;*/
float GYRO_TOL = 10.00;
//// GYROSCOPE END

//// LED AND BUZZER ////
#define LED_PIN 2
#define BUZZER 3
float G = 783.99;
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

// time vars
float prevTime; // in milliseconds
float currTime; // IN SECONDS

/////// PATH CONSTANTS ///////
float LEG_1_DISTANCE = 45.0; // in YARDS
float LEG_2_DISTANCE = 0.0;
float LEG_3_DISTANCE = 0.0;
float totalDistTraveled = 0.0; // distance counter
float deltaT;   // in milliseconds
float UGV_POWER = 18.40;

bool UGV_WAS_TAGGED = false;

void setup() {
  Serial.begin(9600);
  //Serial.print("in setup\n");

  //// INITIALIZATIONS ////
  currTime = millis(); // init time
  init_motors();
  init_encoder();
  //gyro.setupGyroscope(); // Initialize the gyroscope
  //gyro_forward_pitch = gyro.getPitch();
}

void loop(){
  //Serial.print("in loop\n");
  
  if(UGV_WAS_TAGGED){
    ugv_stop();
    tag_sequence();
    exit(0);
  }else{

    if(totalDistTraveled < LEG_1_DISTANCE){
      ugv_forward(UGV_POWER);
      //goStraight(UGV_POWER, GYRO_TOL);
    }else{
      ugv_stop();
    }
    // updates
    updateTime(); // update time vars and deltaT
    totalDistTraveled += enc_speed * deltaT; //update distance traveled
    checkHit();
    /*
    //// GYROSCOPE START
    if(millis()%1000==0){
      curr_pitch = gyro.getPitch();
    }
    */
    
    //// GYROSCOPE END
  }

}

//// MOTOR FUNCTIONS ////
void init_motors(){
  // init all motors in motor array
  for(int i=0;i<4;i++){
    motors[i].initialize();
  }

  float delta = -1.0;
  for(int i=0; i<4; i++){ // manually adjust motor power vals
    if(i==0){ // FL
      motors[i].update_power_mod(-0.41);
    }
    if(i==1){ // FR
      motors[i].update_power_mod(2.15+delta);
    }
    if(i==2){ // BL
      motors[i].update_power_mod(0);
    }
    if(i==3){ // BR
      motors[i].update_power_mod(0.73+delta);
    }
  }
}

void init_encoder(){
  /*
  Initialize Encoder
  */
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt(ENCODER_FR), doCountEnc, CHANGE);  // increase counter when speed sensor pin goes High
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
////////////////////

void updateTime(){
  prevTime = currTime; 
  currTime = millis();
  deltaT = (currTime - prevTime)/1000;
}

void goStraight(float power, float gyro_tol){
  // check for straightness and adjust accordingly then move
  //float curr_pitch = gyro.getPitch();
  /*
  if(curr_pitch > (gyro_forward_pitch+GYRO_TOL)){ // ugv veered right
    for(int i=0; i<4; i++){
      if(motors[i].getOnRight()){
        motors[i].update_power_mod(10);
      }
    }
  }else if(curr_pitch < (gyro_forward_pitch+GYRO_TOL)){
    for(int i=0; i<4; i++){
      if(motors[i].getOnRight()){
        motors[i].update_power_mod(-10);
      }
    }
  }*/
  ugv_forward(UGV_POWER);
}

////////// GYRO FUNCTIONS /////////////
void turn_right(){
  ugv_stop();
  delay(1000);
  float power = 40;
  for(int i=0;i<4;i++){
    if(!motors[i].getOnRight()){
      motors[i].forward(power);
    }
  }
  delay(4000);
  ugv_stop();
}

void turn_left(){
  float power = 40;
  for(int i=0;i<4;i++){
    if(motors[i].getOnRight()){
      motors[i].forward(power);
    }
  }
  delay(2700);
  ugv_stop();
}

/*
void turn_right(float power){
  float gstart_x, gstart_y, gstart_z = gyro.get_xyz();
  float curr_x = gstart_x; // or y or z
  while (curr_x < (gstart_x+90.0)){
    for (int i=0; i<4; i++){
      if(!motors[i].getOnRight){
        motors[i].forward(power);
      }
    }
  }
}

void turn_left(float power){
  float gstart_x, gstart_y, gstart_z = gyro.get_xyz();
  float curr_x = gstart_x; // or y or z
  while (curr_x > (gstart_x-90.0)){
    for(int i=0; i<4; i++){
      if(motors[i].getOnRight){
        motors[i].forward(power);
      }
    }
  }
}
*/

////////// LED AND BUZZER //////////
void tag_sequence(){
  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER, G);
  delay(60000);
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER);
}


///// WATER SENSOR /////
void checkHit(){
  if(ws1.readSensor() > 300){
    UGV_WAS_TAGGED = true;
    //Serial.print("UGV_WAS_TAGGED: ");
    //Serial.print(UGV_WAS_TAGGED);
    //Serial.print("\n");
  }
}