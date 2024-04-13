#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>

class Gyroscope {
  private:
    float gyro_x, gyro_y, gyro_z;
    long acc_x, acc_y, acc_z, acc_total_vector;
    int temperature;
    long gyro_x_cal, gyro_y_cal, gyro_z_cal;
    long loop_timer;
    float angle_pitch, angle_roll;
    int angle_pitch_buffer, angle_roll_buffer;
    boolean set_gyro_angles;
    float angle_roll_acc, angle_pitch_acc;
    float angle_pitch_output, angle_roll_output;

    void read_mpu_6050_data();
    void setup_mpu_6050_registers();

  public:
    Gyroscope();
    //void setup();
    //void loop();
    float get_xyz();
    void setupGyroscope();
    void readGyroscopeData();
    void setupMPU6050Registers();
    void calibrateGyroscope();
    void calculateAngles();
    float getPitch();
};



#endif
