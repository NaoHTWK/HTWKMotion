#pragma once

#include <cmath>

#include <point_3d.h>
#include <stl_ext.h>
#include <position.h>

class Odometry{
  public:
    void apply(float dx, float dy, float angleYaw) {
        //TODO: Add dx/dy properly when rotating.
        pos.x += dx;
        pos.y += dy;
        float angleDiff = normalizeRotation(angleYaw - lastAngleYaw);
        lastAngleYaw = angleYaw;
        pos.a += angleDiff;
    }
    Position getMovementVec() {
        Position v{pos.x, pos.y, pos.a};
        pos=Position();
        return v;
    }

  private:
    float lastAngleYaw=0;
    Position pos;
};
