#ifndef POSITIONFEET_H
#define POSITIONFEET_H
#include <utility>

#include "LegPosition.h"
#include "point_3d.h"

namespace kinematics {

class PositionFeet {
public:
    const LegPosition leftLeg;
    const LegPosition rightLeg;
    std::pair<point_3d, point_3d> calcPosOfFeet(float torsoAngleX, float torsoAngleY);
    std::pair<float, float> calcTorsoAnglesRightLeg();
    std::pair<float, float> calcTorsoAnglesLeftLeg();

    PositionFeet(const LegPosition& _leftLeg, const LegPosition& _rightLeg) : leftLeg(_leftLeg), rightLeg(_rightLeg) {}
};
}  // namespace kinematics
#endif
