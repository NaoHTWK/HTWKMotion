#pragma once

namespace kinematics {

struct LegPositionPlain {
    bool isLeftLeg{};
    float x{};
    float y{};
    float z{};
    float alpha{};
    float beta{};
};

}  // namespace kinematics
