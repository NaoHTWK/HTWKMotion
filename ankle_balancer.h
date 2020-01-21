#pragma once

#include <exponential_moving_average.h>
#include <imu.h>

class AnkleBalancer {
  public:
    void proceed(const YPR& gyro);
    float pitch = 0;
    float roll = 0;
    EMA gyroPitch{0.f, 0.7f};
    EMA gyroRoll{0.f, 0.7f};

  private:
    float forward_gyro_gain = 0.05f;
    float backward_gyro_gain = 0.1f;
    float roll_gyro_gain = 0.06f;

};
