#ifndef FORWARDKINEMATIC_H_
#define FORWARDKINEMATIC_H_
#include <Joints.h>
#include <joints.h>
#include <PositionFeet.h>
namespace kinematics {

class ForwardKinematic {
public:
	static PositionFeet calcPose(const ::Joints &j);
	static PositionFeet calcPose(const Joints &j);

	ForwardKinematic() = delete;
private:
	static LegPosition calcPose(const LegJoints &j, const float &hipYawPitch);
};

} /* namespace kinematics */

#endif /* FORWARDKINEMATIC_H_ */
