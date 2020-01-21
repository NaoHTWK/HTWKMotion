#include "sit_motion.h"

constexpr LegJointAngles SitMotion::sit_angles;
constexpr LegJointAngles SitMotion::stand_angles;

LegJoints SitMotion::getUp(const LegJoints& legs, const AnkleBalancer& ankle_balancer, ArmController* arm_controller) {
    LegJoints result;
    if (sitting == 1.f)
        init_sitting_angles = extract_angles(legs);
    sitting = std::max(0.f, sitting - 0.005f);
    if (sitting < 0.5f)
        arm_controller->request = ArmController::ArmRequest::BACK;
    angle_interpolation(stand_angles, init_sitting_angles, sitting, &result);
    set_stiffness(sitting == 0.f ? 0.3f : 0.8f, &result);
    result[LAnklePitch].angle += ankle_balancer.pitch / 2.f;
    result[RAnklePitch].angle += ankle_balancer.pitch / 2.f;
    return result;
}

LegJoints SitMotion::sitDown(const LegJoints& legs, const AnkleBalancer &ankle_balancer, ArmController* arm_controller) {
    LegJoints result;
    if (sitting == 0.f)
        init_standing_angles = extract_angles(legs);
    sitting = std::min(1.f, sitting + 0.005f);
    angle_interpolation(init_standing_angles, sit_angles, sitting, &result);
    if (sitting == 1.f) {
        arm_controller->request = ArmController::ArmRequest::UNSTIFF;
        set_stiffness(0.f, &result);
        result[LHipPitch].stiffness = 0.3f;
        result[RHipPitch].stiffness = 0.3f;
    } else {
        arm_controller->request = sitting > 0.25f ? ArmController::ArmRequest::FRONT : ArmController::ArmRequest::BACK;
        set_stiffness(0.8f, &result);
    }
    result[LAnklePitch].angle += ankle_balancer.pitch / 2.f;
    result[RAnklePitch].angle += ankle_balancer.pitch / 2.f;
    return result;
}
