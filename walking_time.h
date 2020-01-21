#pragma once

#include <point_2d.h>
#include <point_3d.h>
#include <position.h>
#include <stl_ext.h>
#include <walking_engine.h>

// Estimates time (in seconds) to walk using omnidirectional walking for short distances and turn-walk-turn for long distances.
inline float walkingTime(const Position& start, const Position& end) {
    Position d = (end - Position(start.point(), 0)).rotated(-start.a);
    float time_omni = WalkingEngine::walkingTime(d.x, d.y, d.a);
    float time_turn = WalkingEngine::walkingTime(0, 0, d.point().to_direction()) + WalkingEngine::walkingTime(d.norm(), 0, 0) +
           WalkingEngine::walkingTime(0, 0, d.rotated(-d.point().to_direction()).a);
    return clamped_linear_interpolation(d.norm(), time_omni, time_turn, 0.4f, 0.8f);
}
