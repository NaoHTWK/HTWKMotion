#pragma once

#include "Joints.h"
#include "PositionPlain.h"
#include "LegPosition.h"
#include "LegJoints.h"
#include "PositionFeet.h"

namespace kinematics {

#define LOWERLEG 0.1029f
#define LOWERLEG2 (LOWERLEG*2.f)
#define LOWERLOWERLEG (LOWERLEG*LOWERLEG)
#define UPPERLEG 0.10f
#define UPPERUPPERLEG (UPPERLEG*UPPERLEG)
#define UPPERLOWERLEG (UPPERLEG*LOWERLEG)
#define UPPERLOWERLEG2 (UPPERLEG*LOWERLEG*2.f)
#define UPPERLOWERLEG2pm1 (1.f/UPPERLOWERLEG2)

#define LEGDISTANCE 0.10f
#define LEGDISTANCE_2 (LEGDISTANCE/2.f)

class InverseKinematics {
public:
	static Joints setFeet(const PositionFeet &position);
	static Joints setFeetRelToPlane(const PositionPlain &position, const float &z);
    InverseKinematics() = delete;
private:
	static LegJoints setFoot(const LegPosition &leg);
}
;

} // namespace kinematics
