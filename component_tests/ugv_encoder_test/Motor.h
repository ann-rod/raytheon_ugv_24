#ifndef Motor_h
#define Motor_h
#include "Arduino.h"
#include "Servo.h"
class Motor{
  public:
    Motor::Motor(int give_pin=0, bool onRight=false);
    void initialize();
    void forward(int power);
    void backward(int power);
    void stop();

  private:
    Servo esc;
    int pin;
    bool onRight;

    void set_esc_power(Servo esc, int power);
};
#endif