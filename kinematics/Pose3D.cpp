#include "Pose3D.h"
#include <cmath>
#include <iostream>
namespace kinematics {
#define cosPi_4 0.70710678118654757274f

Pose3D::Pose3D() {
    pose[0][0] = 1;
    pose[0][1] = 0;
    pose[0][2] = 0;
    pose[0][3] = 0;

    pose[1][0] = 0;
    pose[1][1] = 1;
    pose[1][2] = 0;
    pose[1][3] = 0;

    pose[2][0] = 0;
    pose[2][1] = 0;
    pose[2][2] = 1;
    pose[2][3] = 0;
}

void Pose3D::print() const {
    for (auto x : pose) {
        for (int y = 0; y < 4; y++) {
            std::cout << x[y] << "\t";  // display the current element out of the array
        }
        std::cout << std::endl;  // when the inner loop is done, go to a new line
    }
    std::cout << std::endl;
}

Pose3D Pose3D::times(const Pose3D& that) const {
    Pose3D ret = Pose3D();

    ret.pose[0][0] = pose[0][0] * that.pose[0][0] + pose[0][1] * that.pose[1][0] + pose[0][2] * that.pose[2][0];
    ret.pose[0][1] = pose[0][0] * that.pose[0][1] + pose[0][1] * that.pose[1][1] + pose[0][2] * that.pose[2][1];
    ret.pose[0][2] = pose[0][0] * that.pose[0][2] + pose[0][1] * that.pose[1][2] + pose[0][2] * that.pose[2][2];
    ret.pose[0][3] = pose[0][0] * that.pose[0][3] + pose[0][1] * that.pose[1][3] + pose[0][2] * that.pose[2][3] + pose[0][3];

    ret.pose[1][0] = pose[1][0] * that.pose[0][0] + pose[1][1] * that.pose[1][0] + pose[1][2] * that.pose[2][0];
    ret.pose[1][1] = pose[1][0] * that.pose[0][1] + pose[1][1] * that.pose[1][1] + pose[1][2] * that.pose[2][1];
    ret.pose[1][2] = pose[1][0] * that.pose[0][2] + pose[1][1] * that.pose[1][2] + pose[1][2] * that.pose[2][2];
    ret.pose[1][3] = pose[1][0] * that.pose[0][3] + pose[1][1] * that.pose[1][3] + pose[1][2] * that.pose[2][3] + pose[1][3];

    ret.pose[2][0] = pose[2][0] * that.pose[0][0] + pose[2][1] * that.pose[1][0] + pose[2][2] * that.pose[2][0];
    ret.pose[2][1] = pose[2][0] * that.pose[0][1] + pose[2][1] * that.pose[1][1] + pose[2][2] * that.pose[2][1];
    ret.pose[2][2] = pose[2][0] * that.pose[0][2] + pose[2][1] * that.pose[1][2] + pose[2][2] * that.pose[2][2];
    ret.pose[2][3] = pose[2][0] * that.pose[0][3] + pose[2][1] * that.pose[1][3] + pose[2][2] * that.pose[2][3] + pose[2][3];

    return ret;
}
void Pose3D::timesEq(const Pose3D& that) {
    float t00 = pose[0][0];
    float t01 = pose[0][1];
    float t02 = pose[0][2];
    float t03 = pose[0][3];
    float t10 = pose[1][0];
    float t11 = pose[1][1];
    float t12 = pose[1][2];
    float t13 = pose[1][3];
    float t20 = pose[2][0];
    float t21 = pose[2][1];
    float t22 = pose[2][2];
    float t23 = pose[2][3];

    pose[0][0] = t00 * that.pose[0][0] + t01 * that.pose[1][0] + t02 * that.pose[2][0];
    pose[0][1] = t00 * that.pose[0][1] + t01 * that.pose[1][1] + t02 * that.pose[2][1];
    pose[0][2] = t00 * that.pose[0][2] + t01 * that.pose[1][2] + t02 * that.pose[2][2];
    pose[0][3] = t00 * that.pose[0][3] + t01 * that.pose[1][3] + t02 * that.pose[2][3] + t03;

    pose[1][0] = t10 * that.pose[0][0] + t11 * that.pose[1][0] + t12 * that.pose[2][0];
    pose[1][1] = t10 * that.pose[0][1] + t11 * that.pose[1][1] + t12 * that.pose[2][1];
    pose[1][2] = t10 * that.pose[0][2] + t11 * that.pose[1][2] + t12 * that.pose[2][2];
    pose[1][3] = t10 * that.pose[0][3] + t11 * that.pose[1][3] + t12 * that.pose[2][3] + t13;

    pose[2][0] = t20 * that.pose[0][0] + t21 * that.pose[1][0] + t22 * that.pose[2][0];
    pose[2][1] = t20 * that.pose[0][1] + t21 * that.pose[1][1] + t22 * that.pose[2][1];
    pose[2][2] = t20 * that.pose[0][2] + t21 * that.pose[1][2] + t22 * that.pose[2][2];
    pose[2][3] = t20 * that.pose[0][3] + t21 * that.pose[1][3] + t22 * that.pose[2][3] + t23;
}

void Pose3D::rotX(float x) {
    float cx = cosf(x);
    float sx = sinf(x);

    float t01 = pose[0][1];
    float t02 = pose[0][2];
    float t11 = pose[1][1];
    float t12 = pose[1][2];
    float t21 = pose[2][1];
    float t22 = pose[2][2];

    pose[0][1] = t01 * cx + t02 * sx;
    pose[0][2] = t01 * -sx + t02 * cx;
    pose[1][1] = t11 * cx + t12 * sx;
    pose[1][2] = t11 * -sx + t12 * cx;
    pose[2][1] = t21 * cx + t22 * sx;
    pose[2][2] = t21 * -sx + t22 * cx;
}

void Pose3D::rotY(float y) {
    float cy = cosf(y);
    float sy = sinf(y);

    float t00 = pose[0][0];
    float t02 = pose[0][2];
    float t10 = pose[1][0];
    float t12 = pose[1][2];
    float t20 = pose[2][0];
    float t22 = pose[2][2];

    pose[0][0] = t00 * cy - t02 * sy;
    pose[0][2] = t00 * sy + t02 * cy;
    pose[1][0] = t10 * cy - t12 * sy;
    pose[1][2] = t10 * sy + t12 * cy;
    pose[2][0] = t20 * cy - t22 * sy;
    pose[2][2] = t20 * sy + t22 * cy;
}

void Pose3D::rotZ(float z) {
    float cz = cosf(z);
    float sz = sinf(z);

    float t00 = pose[0][0];
    float t01 = pose[0][1];
    float t10 = pose[1][0];
    float t11 = pose[1][1];
    float t20 = pose[2][0];
    float t21 = pose[2][1];

    pose[0][0] = t00 * cz + t01 * sz;
    pose[0][1] = t00 * -sz + t01 * cz;

    pose[1][0] = t10 * cz + t11 * sz;
    pose[1][1] = t10 * -sz + t11 * cz;

    pose[2][0] = t20 * cz + t21 * sz;
    pose[2][1] = t20 * -sz + t21 * cz;
}

void Pose3D::transX(float x) {
    pose[0][3] += pose[0][0] * x;
    pose[1][3] += pose[1][0] * x;
    pose[2][3] += pose[2][0] * x;
}

void Pose3D::transY(float y) {
    pose[0][3] += pose[0][1] * y;
    pose[1][3] += pose[1][1] * y;
    pose[2][3] += pose[2][1] * y;
}

void Pose3D::transZ(float z) {
    pose[0][3] += pose[0][2] * z;
    pose[1][3] += pose[1][2] * z;
    pose[2][3] += pose[2][2] * z;
}

void Pose3D::transXYZ(float x, float y, float z) {
    pose[0][3] += pose[0][0] * x + pose[0][1] * y + pose[0][2] * z;
    pose[1][3] += pose[1][0] * x + pose[1][1] * y + pose[1][2] * z;
    pose[2][3] += pose[2][0] * x + pose[2][1] * y + pose[2][2] * z;
}

void Pose3D::transXYZ(const point_3d& trans) {
    transXYZ(trans.x, trans.y, trans.z);
}

point_3d Pose3D::getTrans() {
    return {pose[0][3], pose[1][3], pose[2][3]};
}

void Pose3D::rotMYX(float y, float x) {

    float cy = cosf(y);
    float sy = sinf(y);
    float cx = cosf(x);
    float sx = sinf(x);

    float r10 = sy * sx;
    float r12 = -sx * cy;
    float r20 = -sy * cx;
    float r22 = cx * cy;

    float t00 = pose[0][0];
    float t01 = pose[0][1];
    float t02 = pose[0][2];
    float t10 = pose[1][0];
    float t11 = pose[1][1];
    float t12 = pose[1][2];
    float t20 = pose[2][0];
    float t21 = pose[2][1];
    float t22 = pose[2][2];

    pose[0][0] = t00 * cy + t01 * (r10) + t02 * (r20);
    pose[1][0] = t01 * cx + t02 * sx;
    pose[2][0] = t00 * sy + t01 * (r12) + t02 * (r22);

    pose[0][1] = t10 * cy + t11 * (r10) + t12 * (r20);
    pose[1][1] = t11 * cx + t12 * sx;
    pose[2][1] = t10 * sy + t11 * (r12) + t12 * (r22);

    pose[0][2] = t20 * cy + t21 * (r10) + t22 * (r20);
    pose[1][2] = t21 * cx + t22 * sx;
    pose[2][2] = t20 * sy + t21 * (r12) + t22 * (r22);
}

void Pose3D::rot45X(int sign) {
    float cx = cosPi_4;
    float sx = sign * cosPi_4;

    float t01 = pose[0][1];
    float t02 = pose[0][2];
    float t11 = pose[1][1];
    float t12 = pose[1][2];
    float t21 = pose[2][1];
    float t22 = pose[2][2];

    pose[0][1] = t01 * cx + t02 * sx;
    pose[0][2] = t01 * -sx + t02 * cx;
    pose[1][1] = t11 * cx + t12 * sx;
    pose[1][2] = t11 * -sx + t12 * cx;
    pose[2][1] = t21 * cx + t22 * sx;
    pose[2][2] = t21 * -sx + t22 * cx;
}

void Pose3D::rotMZXY(float z, float x, float y) {
    float cy = cosf(y);
    float sy = sinf(y);
    float cx = cosf(x);
    float sx = sinf(x);
    float cz = cosf(z);
    float sz = sinf(z);

    float r02 = sy * cx;
    float r10 = sz * cx;
    float r11 = cz * cx;
    float r22 = cy * cx;
    float r00 = cy * cz + sz * sx * sy;
    float r01 = -sz * cy + sx * sy * cz;
    float r20 = -sy * cz + sx * cy * sz;
    float r21 = sz * sy + sx * cy * cz;

    float t00 = pose[0][0];
    float t01 = pose[0][1];
    float t02 = pose[0][2];
    float t10 = pose[1][0];
    float t11 = pose[1][1];
    float t12 = pose[1][2];
    float t20 = pose[2][0];
    float t21 = pose[2][1];
    float t22 = pose[2][2];

    pose[0][0] = t00 * r00 + t01 * r10 + t02 * r20;
    pose[1][0] = t00 * r01 + t01 * r11 + t02 * r21;
    pose[2][0] = t00 * r02 - t01 * sx + t02 * r22;

    pose[0][1] = t10 * r00 + t11 * r10 + t12 * r20;
    pose[1][1] = t10 * r01 + t11 * r11 + t12 * r21;
    pose[2][1] = t10 * r02 - t11 * sx + t12 * r22;

    pose[0][2] = t20 * r00 + t21 * r10 + t22 * r20;
    pose[1][2] = t20 * r01 + t21 * r11 + t22 * r21;
    pose[2][2] = t20 * r02 - t21 * sx + t22 * r22;
}
point_3d Pose3D::times(const point_3d& point) const {
    point_3d ret;
    ret.x = pose[0][0] * point.x + pose[0][1] * point.y + pose[0][2] * point.z + pose[0][3];
    ret.y = pose[1][0] * point.x + pose[1][1] * point.y + pose[1][2] * point.z + pose[1][3];
    ret.z = pose[2][0] * point.x + pose[2][1] * point.y + pose[2][2] * point.z + pose[2][3];
    return ret;
}
} /* namespace kinematics */
