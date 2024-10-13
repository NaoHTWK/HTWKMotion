#include "arm_controller.h"

#include <stl_ext.h>

#include <cmath>

using namespace std;

constexpr std::array<float, 5> ArmController::shoulder_pitch;
constexpr std::array<float, 5> ArmController::shoulder_roll;
constexpr std::array<float, 5> ArmController::elbow_yaw;
constexpr std::array<float, 5> ArmController::elbow_roll;

ArmJoints ArmController::proceed() {
    if (request == ArmRequest::FRONT || request == ArmRequest::UNSTIFF) {
        state = limitpm1(state - 0.015f);
    } else if (request == ArmRequest::SIDE) {
        if (state > 0) {
            state = limit01(state - 0.015f);
        } else if (state < 0) {
            state = -limit01(-state - 0.015f);
        }
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

void ArmController::updateState() {
    state = 1.f;
}

void ArmController::updateState(ArmJoints arms) {
    float steps = (2.f / 0.015f) / 4.f;
    float angle = arms[LShoulderPitch].angle;
    float diff, totalDiff, offset;
    if (angle > shoulder_pitch[0]) {
        diff = shoulder_pitch[1] - angle;
        totalDiff = shoulder_pitch[1] - shoulder_pitch[0];
        offset = 1.f;
    } else if ((shoulder_pitch[0] >= angle) && (angle > shoulder_pitch[2])) {
        diff = shoulder_pitch[0] - angle;
        totalDiff = shoulder_pitch[0] - shoulder_pitch[2];
        offset = 1.f - steps;
    } else if ((shoulder_pitch[2] >= angle) && (angle > shoulder_pitch[3])) {
        diff = shoulder_pitch[2] - angle;
        totalDiff = shoulder_pitch[2] - shoulder_pitch[3];
        offset = 1.f - (steps * 2.f);
    } else {
        return;
    }
    float ratio = diff / totalDiff;
    float factor = ratio * steps;
    state = limitpm1(offset - (factor * 0.015f));
}
