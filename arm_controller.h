#pragma once

#include <array>

#include <joints.h>

class ArmController {
  public:
    enum class ArmRequest { UNSTIFF, FRONT, BACK };
    ArmRequest request = ArmRequest::UNSTIFF;
    bool isFront() { return state <= -1.f; }
    bool isBack() { return state >= 1.f; }
    ArmJoints proceed();

  private:
    float state = -1.f;
    static constexpr std::array<float, 5> shoulder_pitch{2.038644f, 2.119946f, 1.641338f, 1.54f, 1.56f};
    static constexpr std::array<float, 5> shoulder_roll{-0.141170f, 0.08f, 0.25f, 0.1f, -0.08f};
    static constexpr std::array<float, 5> elbow_yaw{0.524586f, 0.285282f, -0.791586f, -1.42f, -1.0};
    static constexpr std::array<float, 5> elbow_roll{-0.687190f, -0.133416f, -0.108872f, -1.48f, -1.15f};
};
