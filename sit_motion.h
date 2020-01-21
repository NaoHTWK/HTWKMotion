#pragma once

#include <ankle_balancer.h>
#include <arm_controller.h>
#include <joints.h>

class SitMotion {
  public:
    LegJoints getUp(const LegJoints& legs, const AnkleBalancer &ankle_balancer, ArmController* arm_controller);
    LegJoints sitDown(const LegJoints& legs, const AnkleBalancer &ankle_balancer, ArmController* arm_controller);
    bool isSitting() const { return sitting >= 1.f; }
    bool isStanding() const { return sitting <= 0.f; }

  private:
    float sitting = 1.f;
    static constexpr LegJointAngles sit_angles{
        0.f, // LHipRoll
        -0.885f, // LHipPitch
        2.2f, // LKneePitch
        -1.24f, // LAnklePitch
        0.f, // LAnkleRoll
        0.f, // HipYawPitch
        0.f, // RHipRoll
        -0.885f, // RHipPitch
        2.2f, // RKneePitch
        -1.24f, // RAnklePitch
        0.f, // RAnkleRoll
    };
    static constexpr LegJointAngles stand_angles{
        0.f, // LHipRoll
        0.f, // LHipPitch
        0.2f, // LKneePitch
        -0.18f, // LAnklePitch
        0.f, // LAnkleRoll
        0.f, // HipYawPitch
        0.f, // RHipRoll
        0.f, // RHipPitch
        0.2f, // RKneePitch
        -0.18f, // RAnklePitch
        0.f, // RAnkleRoll
    };
    LegJointAngles init_sitting_angles = sit_angles;
    LegJointAngles init_standing_angles = stand_angles;
};
