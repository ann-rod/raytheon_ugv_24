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
    bool getOnRight();
    void update_power_mod(float pm_adjust);
    float getPowerMod();

  private:
    Servo esc;
    int pin;
    bool onRight;
    float power_mod;
    void set_esc_power(Servo esc, int power);
};
#endif