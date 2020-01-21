#ifndef LEGPOSITION_H
#define LEGPOSITION_H

namespace kinematics {

struct LegPosition {
    const bool isLeftLeg;
    const float x;
    const float y;
    const float z;
    const float rX;
    const float rY;
    const float rZ;

    LegPosition(const bool _isLeftLeg, const float _x, const float _y, const float _z, const float _rX, const float _rY, const float _rZ)
        : isLeftLeg(_isLeftLeg), x(_x), y(_y), z(_z), rX(_rX), rY(_rY), rZ(_rZ) {}
};
}  // namespace kinematics
#endif
