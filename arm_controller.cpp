#include "arm_controller.h"

#include <cmath>

#include <stl_ext.h>

using namespace std;

constexpr std::array<float, 5> ArmController::shoulder_pitch;
constexpr std::array<float, 5> ArmController::shoulder_roll;
constexpr std::array<float, 5> ArmController::elbow_yaw;
constexpr std::array<float, 5> ArmController::elbow_roll;

ArmJoints ArmController::proceed() {
    if (request == ArmRequest::FRONT || request == ArmRequest::UNSTIFF) {
        state = limitpm1(state - 0.015f);
    } else {
        state = limitpm1(state + 0.015f);
    }
    ArmJoints result;
    int range = (shoulder_pitch.size() - 1) / 2;
    for (int i = -range; i <= range; i++) {
      float g = limit01(1.f - abs(state + static_cast<float>(i) / range) * range);
      result[LShoulderPitch].angle += shoulder_pitch[range + i] * g;
      result[LShoulderRoll].angle += shoulder_roll[range + i] * g;
      result[LElbowYaw].angle += elbow_yaw[range + i] * g;
      result[LElbowRoll].angle += elbow_roll[range + i] * g;
    }
    result[LWristYaw].angle = -90_deg;
    result[LHand].angle = 0.f;
    if (request == ArmRequest::UNSTIFF && state <= -1) {
        set_stiffness(-1.f, &result);
    } else {
        set_stiffness(0.7f, &result);
    }
    mirrorLtoR(&result);
    return result;
}
