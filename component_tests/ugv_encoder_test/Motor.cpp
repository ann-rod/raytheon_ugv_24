#include "Arduino.h"
#include "Motor.h"

Servo esc;
int pin;
bool onRight;

Motor::Motor(int give_pin=0, bool rightTag=false){
  // create motor object
  pin = give_pin;
  onRight = rightTag;
}

void Motor::initialize(){
  // initialize motor object
  esc.attach(pin);
}

void Motor::set_esc_power(Servo esc, int power){
  // set motor power % (negative is backwards)
  power = constrain(power, -100, 100);
   int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
}

void Motor::forward(int power){
  // move motor in ugv-forward direction
  if(onRight){power=-power;}
  set_esc_power(esc,power);
}

void Motor::backward(int power){
  // move motor in ugv-backward direction
  if(onRight){power=-power;}
  set_esc_power(esc,-power);
}

void Motor::stop(){
  // stop motor
  set_esc_power(esc, 0);
}
