#ifndef POSE3D_H_
#define POSE3D_H_
#include "point_3d.h"

namespace kinematics {

class Pose3D {

public:
	Pose3D();
    void transX(float x);
    void transY(float y);
    void transZ(float z);
    void transXYZ(float x, float y, float z);
	void transXYZ(const point_3d &trans);
	point_3d getTrans();

    void rotX(float x);
    void rot45X(int sign);

    void rotY(float y);
    void rotZ(float z);
    void rotMZXY(float z, float x, float y);

    void rotMYX(float y, float x);

	void print() const;
	point_3d times(const point_3d& point) const;

	void timesEq(const Pose3D &that);
	float pose[3][4]{};
	Pose3D times(const Pose3D &that) const;
};

} /* namespace kinematics */

#endif /* POSE3D_H_ */
