#include "PositionFeet.h"

#include <cmath>
#include <cstdio>

#include "Pose3D.h"

namespace kinematics {

std::pair<point_3d, point_3d> PositionFeet::calcPosOfFeet(float torsoAngleX,
        float torsoAngleY) {
	float rb = torsoAngleY;
	float ra = torsoAngleX;
	float sum = std::fabs(ra) + std::fabs(rb);
	if (sum > M_PI_2) {
		float d = M_PI_2 / sum;
		rb *= d;
		ra *= d;
	}
	float tao = sinf(ra) / cosf(rb);
	if(cosf(rb)==0){
//		printf("angles: %f,%f\n",torsoAngleY,torsoAngleX);
//		printf("NaN in Position.cpp TELL THOMAS!!!\n");
//		system("wall \"NaN in Position.cpp TELL THOMAS!!!\"");
		tao=0;
//		sleep(1);
//		exit(-1);
	}
	if(tao!=tao){
		tao=0;
	}
	if (tao > 1) {
		tao = 1;
	}
	if (tao < -1) {
		tao = -1;
	}
	Pose3D rottemp = Pose3D();
	rottemp.rotMYX(rb, -asinf(tao));

	point_3d ergL = rottemp.times(point_3d(leftLeg.x, leftLeg.y, leftLeg.z));
	point_3d ergR = rottemp.times(point_3d(rightLeg.x, rightLeg.y, rightLeg.z));
	return std::make_pair(ergL, ergR);
}

std::pair<float, float> PositionFeet::calcTorsoAnglesRightLeg() {
	float rx = -asinf(sinf(rightLeg.rX) * cosf(rightLeg.rY));
	return std::make_pair(rx, rightLeg.rY);
}

std::pair<float, float> PositionFeet::calcTorsoAnglesLeftLeg() {
	float rx = -asinf(sinf(leftLeg.rX) * cosf(leftLeg.rY));
	return std::make_pair(rx, leftLeg.rY);

}
}
