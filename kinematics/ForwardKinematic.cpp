#include "ForwardKinematic.h"
#include "Pose3D.h"
#include <cmath>
#include <cstdio>
#include <unistd.h>

#include "InverseKinematics.h"
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace kinematics {

    PositionFeet ForwardKinematic::calcPose(const Joints &j) {
        return {calcPose(j.leftLeg, j.hipYawPitch), calcPose(j.rightLeg, j.hipYawPitch)};
    }

    LegPosition ForwardKinematic::calcPose(const LegJoints &j,
                                           const float &hipYawPitch) {
        Pose3D pl = Pose3D();
        int s = (j.isLeftLeg) ? 1 : -1;
        pl.transY(-s * LEGDISTANCE_2);
        pl.rotX(s * M_PI_4);
        pl.rotZ(s * hipYawPitch);
        pl.rotX(-s * M_PI_4 + j.hipRoll);
        pl.rotY(-j.hipPitch);
        pl.transZ(UPPERLEG);
        pl.rotY(-j.kneePitch);
        pl.transZ(LOWERLEG);
        pl.rotY(-j.anklePitch);
        pl.rotX(j.ankleRoll);
        if (pl.pose[1][2] < -1 || pl.pose[1][2] > 1) {
            int rc = system(R"(wall "NaN in InverseKinematics.cpp tempL TELL THOMAS!!!")");
            if (rc != 0) {
                printf("NaN in InverseKinematics.cpp tempL TELL THOMAS!!!\n");
                fflush(stdout);
            }
            exit(-1); // Log

        }
        float rX = -asinf(pl.pose[1][2]);
        float cRX = cosf(rX);
        if (cRX == 0) {
            int rc = system(R"(wall "NaN in InverseKinematics.cpp tempL TELL THOMAS!!!")");
            if (rc != 0) {
                printf("NaN in InverseKinematics.cpp tempL TELL THOMAS!!!\n");
                fflush(stdout);
            }
            exit(-1); // Log
        }
        float rZ = asinf(pl.pose[1][0] / cRX);
        float rY = asinf(pl.pose[0][2] / cRX);
        return LegPosition(true, pl.pose[0][3], pl.pose[1][3], pl.pose[2][3], rX, rY, rZ);
    }

    PositionFeet ForwardKinematic::calcPose(const ::Joints &j) {
        kinematics::Joints jointsKin = kinematics::Joints(
                kinematics::LegJoints(true,
                                      j.legs[LegJointNames::HipYawPitch].angle,
                                      j.legs[LegJointNames::LHipRoll].angle,
                                      j.legs[LegJointNames::LHipPitch].angle,
                                      j.legs[LegJointNames::LKneePitch].angle,
                                      j.legs[LegJointNames::LAnklePitch].angle,
                                      j.legs[LegJointNames::LAnkleRoll].angle,
                                      false),
                kinematics::LegJoints(false,
                                      j.legs[LegJointNames::HipYawPitch].angle,
                                      j.legs[LegJointNames::RHipRoll].angle,
                                      j.legs[LegJointNames::RHipPitch].angle,
                                      j.legs[LegJointNames::RKneePitch].angle,
                                      j.legs[LegJointNames::RAnklePitch].angle,
                                      j.legs[LegJointNames::RAnkleRoll].angle,
                                      false));
        return calcPose(jointsKin);
    }

} /* namespace kinematics */
