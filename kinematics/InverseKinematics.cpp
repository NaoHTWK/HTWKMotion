#include "InverseKinematics.h"
#include "Pose3D.h"
#include "iostream"
#include <cmath>
#include <stl_ext.h>

namespace kinematics {

Joints InverseKinematics::setFeet(const PositionFeet &position) {
    return {setFoot(position.leftLeg), setFoot(position.rightLeg)};
}

Joints InverseKinematics::setFeetRelToPlane(const PositionPlain &position, const float &z) {
	float alphaL = position.leftLeg.alpha;
	float betaL = -position.leftLeg.beta;
	float gamma = position.gamma;
	float caL = cosf(alphaL);
	float saL = sinf(alphaL);
	float cbL = cosf(betaL);
	float sbL = sinf(betaL);
	float tempL = sqrtf(std::max(0.f,1.f - saL * saL - sbL * sbL));

	if(tempL!=tempL){
        int rc = system(R"(wall "NaN in InverseKinematics.cpp tempL TELL THOMAS!!!")");
        if(rc != 0) {
            printf("NaN in InverseKinematics.cpp tempL TELL THOMAS!!!\n");
            fflush(stdout);
        }
        exit(-1); // Log
	}

	float alphaR = position.rightLeg.alpha;
	float betaR = -position.rightLeg.beta;
	float caR = cosf(alphaR);
	float saR = sinf(alphaR);
	float cbR = cosf(betaR);
	float sbR = sinf(betaR);
	float tempR = sqrtf(std::max(0.f,1 - saR * saR - sbR * sbR));

	float xl = position.leftLeg.x * caL; // x-shift (left leg x)
	xl += position.leftLeg.z * saL; // x-shift (left leg z)
	float yl = position.leftLeg.y * cbL; // y-shift (left leg y)
	yl -= position.leftLeg.z * sbL; // y-shift (left leg z)
	float zl = z; // z-shift (plane)
	zl -= position.leftLeg.x * saL; // z-shift (left leg x)
	zl += position.leftLeg.y * sbL; // z-shift (left leg y)
	zl += position.leftLeg.z * tempL; // z-shift (left leg z)

	float xr = position.rightLeg.x * caR; // x-shift (right leg x)
	xr += position.rightLeg.z * saR; // x-shift (right leg z)
	float yr = position.rightLeg.y * cbR; // y-shift (right leg y)
	yr -= position.rightLeg.z * sbR; // y-shift (right leg z)
	float zr = z; // z-shift (plane)
	zr -= position.rightLeg.x * saR; // z-shift (right leg x)
	zr += position.rightLeg.y * sbR; // z-shift (right leg y)
	zr += position.rightLeg.z * tempR; // z-shift (right leg z)
	if(std::cos(alphaL)!=0){
        betaL = std::asin(limitpm1(std::sin(betaL) / std::cos(alphaL)));
	}else{
		betaL = 0;
	}
	if(std::cos(alphaR)!=0){
        betaR = std::asin(limitpm1(std::sin(betaR) / std::cos(alphaR)));
	}else{
		betaR = 0;
	}
	if(betaL!=betaL||betaR!=betaR){
        int rc = system(R"(wall "NaN in InverseKinematics.cpp betaL/R TELL THOMAS!!!")");
        if(rc != 0) {
            printf("NaN in InverseKinematics.cpp betaL/R TELL THOMAS!!!\n");
            fflush(stdout);
        }
        exit(-1); // Log
	}
    return setFeet(PositionFeet(
                       LegPosition(true, xl, yl, zl, betaL, alphaL, gamma),
                       LegPosition(false, xr, yr, zr, betaR, alphaR, -gamma)));
}

LegJoints InverseKinematics::setFoot(const LegPosition &leg) {
	int s = (leg.isLeftLeg) ? 1 : -1;
	bool clipping = false;
	Pose3D hipRelToFootOrth = Pose3D();
    hipRelToFootOrth.rotMZXY(leg.rZ, leg.rX, leg.rY);
    hipRelToFootOrth.transXYZ(-leg.x, -leg.y - s * LEGDISTANCE_2, -leg.z);
    hipRelToFootOrth.rot45X(s);

	point_3d tran = hipRelToFootOrth.getTrans();
	float transyz2 = tran.y * tran.y + tran.z * tran.z;
	float ltrans2 = tran.x * tran.x + transyz2;

	float cosKnee = (UPPERUPPERLEG + LOWERLOWERLEG - ltrans2)
			* UPPERLOWERLEG2pm1;
	if (cosKnee > 1) {
		cosKnee = 1;
		clipping = true;
	} else {
		if (cosKnee < -1) {
			cosKnee = -1;
			clipping = true;
		}
	}
    float kneePitch = M_PIf - acosf(cosKnee);

	float cosLowerLeg = (LOWERLOWERLEG + ltrans2 - UPPERUPPERLEG)
			/ (LOWERLEG2 * sqrtf(ltrans2));
	if(cosLowerLeg!=cosLowerLeg||ltrans2<=0){
        int rc = system(R"(wall "NaN in InverseKinematics.cpp cosLowerLeg/ltrans2 TELL THOMAS!!!")");
        if(rc != 0) {
            printf("NaN in InverseKinematics.cpp cosLowerLeg/ltrans2 TELL THOMAS!!!\n");
            printf("%f %f\n", cosLowerLeg, ltrans2);
            fflush(stdout);
        }
        exit(-1); // Log
	}

	if (cosLowerLeg > 1) {
		cosLowerLeg = 1;
		clipping = true;
	} else {
		if (cosLowerLeg < -1) {
			cosLowerLeg = -1;
			clipping=true;
		}
	}

	float dFootPitch1 = acosf(cosLowerLeg);
	float dFootPitch2 = atan2f(tran.x, sqrtf(transyz2));
	float anklePitch = -(dFootPitch1 + dFootPitch2);
	float ankleRoll = -atan2f(tran.y, -tran.z);

	Pose3D thigh2hipO = Pose3D();
	thigh2hipO.rotMYX(anklePitch + kneePitch, -ankleRoll);
	thigh2hipO.timesEq(hipRelToFootOrth);

    float hipYawPitch = -s * std::atan2(thigh2hipO.pose[1][0], thigh2hipO.pose[1][1]);
    float hipPitch = std::atan2(thigh2hipO.pose[0][2], thigh2hipO.pose[2][2]);
    float hipRoll = std::asin(thigh2hipO.pose[1][2]) + s * static_cast<float>(M_PI_4);
    return LegJoints(leg.isLeftLeg, hipYawPitch, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll, clipping);
}

} /* namespace kinematics */
