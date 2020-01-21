#ifndef POSITIONPLAIN_H
#define POSITIONPLAIN_H
#include <utility>
#include "LegPositionPlain.h"
#include "point_3d.h"
namespace kinematics {

struct PositionPlain {
	const LegPositionPlain leftLeg;
	const LegPositionPlain rightLeg;
	const float gamma;

	PositionPlain(const LegPositionPlain _leftLeg, const LegPositionPlain _rightLeg, const float _gamma) :
			leftLeg(_leftLeg), rightLeg(_rightLeg), gamma(_gamma) {
	}
};
}
#endif
