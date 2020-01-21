#ifndef JOINTS_H
#define JOINTS_H

#include <utility>

#include "LegJoints.h"
namespace kinematics {

struct Joints {
	const LegJoints leftLeg;
	const LegJoints rightLeg;
	const float hipYawPitch;

    Joints(LegJoints _leftLeg, LegJoints _rightLeg)
        : leftLeg(_leftLeg)
        , rightLeg(_rightLeg)
        , hipYawPitch((leftLeg.hipYawPitch + rightLeg.hipYawPitch) / 2.0f) {
	}
};
}
#endif
