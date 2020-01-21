#ifndef LEGPOSITIONPLAIN_H
#define LEGPOSITIONPLAIN_H
namespace kinematics {

struct LegPositionPlain {
    const bool isLeftLeg;
    const float x;
    const float y;
    const float z;
    const float alpha;
    const float beta;
    LegPositionPlain(const bool _isLeftLeg, const float _x, const float _y, const float _z, const float _alpha, const float _beta)
        : isLeftLeg(_isLeftLeg), x(_x), y(_y), z(_z), alpha(_alpha), beta(_beta) {}
};
}  // namespace kinematics
#endif
