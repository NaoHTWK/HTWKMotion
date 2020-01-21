#include "ankle_balancer.h"

void AnkleBalancer::proceed(const YPR &gyro) {
    gyroPitch += gyro.pitch;
    gyroRoll += gyro.roll;
    pitch = gyroPitch * (gyroPitch > 0.f ? forward_gyro_gain : backward_gyro_gain);
    roll = gyroRoll * roll_gyro_gain;
}
