#pragma once

#include <utility>
#include "LegPositionPlain.h"

namespace kinematics {

struct PositionPlain {
    LegPositionPlain leftLeg;
    LegPositionPlain rightLeg;
    float gamma{};
};

}
