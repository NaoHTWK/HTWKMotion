#include "walking_engine.h"

#include <cmath>
#include <iostream>
#include <vector>

#include <InverseKinematics.h>
#include <LegPositionPlain.h>
#include <PositionPlain.h>
#include <joints.h>
#include <stl_ext.h>

using namespace std;
using namespace htwk;

float WalkingEngine::max_forward = 0.28f;
float WalkingEngine::max_backward = 0.25f;
float WalkingEngine::max_strafe = 0.35f;
float WalkingEngine::max_turn = 1.7f;
float WalkingEngine::velocity_combination_damping = 0.3f;

float parabolicReturn(float f) {
    if (f < 0.25f) {
        return 8.f * f * f;
    } else if (f < 0.75f) {
        float v = f - 0.5f;
        return 1.f - 8.f * v * v;
    } else {
        float v = 1.f - f;
        return 8.f * v * v;
    }
}

float WalkingEngine::get_spline_value(float t, float* spline) {
    for (int j = 1; j < NUM_SPLINE_POINTS; j++) {
        for (int i = 0; i < NUM_SPLINE_POINTS - j; i++) {
            spline[j * NUM_SPLINE_POINTS + i] =
                    spline[(j - 1) * NUM_SPLINE_POINTS + i] * (1 - t) + spline[(j - 1) * NUM_SPLINE_POINTS + i + 1] * t;
        }
    }
    return (spline[(NUM_SPLINE_POINTS - 1) * NUM_SPLINE_POINTS]);
}

// positive is left foot
float WalkingEngine::calcSupportFoot(FSR fsr) {
    static constexpr float max_pressure = 5.0f;
    static const FSR weights{.left = {0.8f, 0.3f, 0.8f, 0.3f}, .right = {-0.3f, -0.8f, -0.3f, -0.8f}};
    float total = 0;
    float weighted = 0;
    for (Foot* f : {&fsr.left, &fsr.right}) {
        for (float* v : {&f->fl, &f->fr, &f->rl, &f->rr}) {
            *v = min(max_pressure, *v);
            float* m = (float*)&fsr_max_values + ((float*)v - (float*)&fsr);
            float* w = (float*)&weights + ((float*)v - (float*)&fsr);
            *m = max(*v, *m);
            if (*m > 0.f) {
                *v /= *m;
                total += *v;
                weighted += *w * *v;
            }
        }
    }
    if (total == 0) {
        return 0;
    }
    return weighted / total;
}

float WalkingEngine::linearParameterChange(float in, float target, float delta) {
    if (in < target) {
        return min(in + delta, target);
    } else if (in > target) {
        return max(in - delta, target);
    }
    return target;
}

void WalkingEngine::detect_and_proceed_phase_switch(const FSR& fsr) {
    float new_support_foot = calcSupportFoot(fsr);
    float new_support_foot_prediction =
            new_support_foot + support_foot_swap_factor * (new_support_foot - last_support_foot_value);
    last_support_foot_value = new_support_foot;

    if (t > max_rel_step_duration * step_duration_factor || support_foot == 0 ||
        (sgn(new_support_foot_prediction) != sgn(support_foot) && t > min_rel_step_duration) ||
        (abs(dx) + abs(dy) + abs(da) < 0.001f && abs(dx_request) + abs(dy_request) + abs(da_request) >= 0.001f)) {
        switch_phase = t < min_rel_step_duration ? 1.f : t;
        support_foot = new_support_foot_prediction;
        if (t > max_rel_step_duration * step_duration_factor &&
            abs(dx_request) + abs(dy_request) + abs(da_request) >= 0.001f) {
            if (step_duration_factor * max_rel_step_duration > 0.95f) {
                step_duration_factor *= 0.9f;
                support_foot = -support_foot;
            }
        } else {
            step_duration_factor = 1;
        }
        t = 0;

        last_dx_left = dx_left;
        last_dx_right = dx_right;
        last_dy_left = dy_left;
        last_dy_right = dy_right;
        last_da_left = da_left;
        last_da_right = da_right;
        if (dx < dx_request * step_duration) {
            dx = min(dx_request * step_duration, dx + max_acceleration_forward * step_duration);
        } else if (dx > dx_request * step_duration) {
            dx = max(dx_request * step_duration, dx - max_deceleration_forward * step_duration);
        }
        if (dy < dy_request * step_duration) {
            dy = min(dy_request * step_duration, dy + max_acceleration_side * step_duration);
        } else if (dy > dy_request * step_duration) {
            dy = max(dy_request * step_duration, dy - max_acceleration_side * step_duration);
        }
        if (da < da_request * step_duration) {
            da = min(da_request * step_duration, da + max_acceleration_turn * step_duration);
        } else if (da > da_request * step_duration) {
            da = max(da_request * step_duration, da - max_acceleration_turn * step_duration);
        }
    }
    if (v_angle < v_angle_request) {
        v_angle = min(v_angle_request, v_angle + max_acceleration_v_angle / frames_per_second);
    } else if (v_angle > v_angle_request) {
        v_angle = max(v_angle_request, v_angle - max_acceleration_v_angle / frames_per_second);
    }
}

void WalkingEngine::update_velocities(float swing_spline_value) {
    if (support_foot > 0) {
        dx_left = last_dx_left + (-dx / 2.f - last_dx_left) * min(t, 1.f);
        dx_right = last_dx_right + (dx / 2.f - last_dx_right) * swing_spline_value;
        if (dy >= 0) {
            dy_left = last_dy_left + (0.f - last_dy_left) * min(t, 1.f);
            dy_right = last_dy_right + (0.f - last_dy_right) * swing_spline_value;
        } else {
            dy_left = last_dy_left + (-dy / 2.f - last_dy_left) * min(t, 1.f);
            dy_right = last_dy_right + (dy / 2.f - last_dy_right) * swing_spline_value;
        }
        if (da >= 0) {
            da_left = last_da_left + (-da / 6.f - last_da_left) * min(t, 1.f);
            da_right = last_da_right + (da / 6.f - last_da_right) * swing_spline_value;
        } else {
            da_left = last_da_left + (-da / 3.f - last_da_left) * min(t, 1.f);
            da_right = last_da_right + (da / 3.f - last_da_right) * swing_spline_value;
        }
    } else {
        dx_left = last_dx_left + (dx / 2.f - last_dx_left) * swing_spline_value;
        dx_right = last_dx_right + (-dx / 2.f - last_dx_right) * min(t, 1.f);
        if (dy >= 0) {
            dy_left = last_dy_left + (dy / 2.f - last_dy_left) * swing_spline_value;
            dy_right = last_dy_right + (-dy / 2.f - last_dy_right) * min(t, 1.f);
        } else {
            dy_left = last_dy_left + (0.f - last_dy_left) * swing_spline_value;
            dy_right = last_dy_right + (0.f - last_dy_right) * min(t, 1.f);
        }
        if (da >= 0) {
            da_left = last_da_left + (da / 3.f - last_da_left) * swing_spline_value;
            da_right = last_da_right + (-da / 3.f - last_da_right) * min(t, 1.f);
        } else {
            da_left = last_da_left + (da / 6.f - last_da_left) * swing_spline_value;
            da_right = last_da_right + (-da / 6.f - last_da_right) * min(t, 1.f);
        }
    }
}

LegJoints WalkingEngine::set_joints(float waddle_right, float waddle_left, const AnkleBalancer& ankle_balancer,
                                    float step_height, const kinematics::Joints& j) {
    LegJoints result;
    result[HipYawPitch].angle = j.hipYawPitch;
    result[LHipRoll].angle = j.leftLeg.hipRoll;
    result[LHipPitch].angle = j.leftLeg.hipPitch;
    result[LKneePitch].angle = j.leftLeg.kneePitch;
    result[LAnklePitch].angle = j.leftLeg.anklePitch;
    result[LAnkleRoll].angle = j.leftLeg.ankleRoll;
    result[RHipRoll].angle = j.rightLeg.hipRoll;
    result[RHipPitch].angle = j.rightLeg.hipPitch;
    result[RKneePitch].angle = j.rightLeg.kneePitch;
    result[RAnklePitch].angle = j.rightLeg.anklePitch;
    result[RAnkleRoll].angle = j.rightLeg.ankleRoll;

    if (step_height == 0) {  // standing
        result[LAnklePitch].angle += ankle_balancer.pitch * 0.5f;
        result[LAnkleRoll].angle += ankle_balancer.roll * 0.5f;
        result[RAnklePitch].angle += ankle_balancer.pitch * 0.5f;
        result[RAnkleRoll].angle += ankle_balancer.roll * 0.5f;
    } else {  // walking
        if (support_foot > 0) {
            result[LAnklePitch].angle += ankle_balancer.pitch;
            result[LAnkleRoll].angle += ankle_balancer.roll;
            result[RAnklePitch].angle += waddle_right;
        } else {
            result[LAnklePitch].angle += waddle_left;
            result[RAnklePitch].angle += ankle_balancer.pitch;
            result[RAnkleRoll].angle += ankle_balancer.roll;

        }
    }
    set_stiffness(dynamic_body_height == body_height_stand ? 0.6f : 0.8f, &result);
    return result;
}

float WalkingEngine::deadband_filter(float min, float value, float max){
        if(value<0) {
            value-=min;
            if(value>0) {
                value=0;
            }
        }
        if(value>0) {
            value-=max;
            if(value<0) {
                value=0;
            }
        }
        return value;
    }

LegJoints WalkingEngine::proceed(const FSR& fsr, float body_pitch, float body_roll, const AnkleBalancer& ankle_balancer, float yaw,
                                 Odometry* odo, ArmController* arm_controller) {
    arm_controller->request = ArmController::ArmRequest::BACK;

    float body_pitch_error=deadband_filter(body_pitch_deadband_min,body_pitch,body_pitch_deadband_max);
    float gyro_pitch_error=deadband_filter(-gyro_deadband,ankle_balancer.gyroPitch,gyro_deadband);

    float step_duration_correction=clamp(-0.04f,-body_pitch_error*dx*step_duration_correction_gain,0.3f);
    float vx_correction=clamp(vx_correction_min,body_pitch_error*vx_correction_bpitch_gain+gyro_pitch_error*vx_correction_gpitch_gain,vx_correction_max);
    float step_height_correction=0;
    vx_correction_smooth=linearParameterChange(vx_correction_smooth,vx_correction,vx_correction_step);

    detect_and_proceed_phase_switch(fsr);

    float stairway_vx_correction=0;

    float step_height = basic_step_height + forward_step_height * abs(dx) / step_duration+step_height_correction;

    if (abs(dx) + abs(dy) + abs(da) < 0.001f ) {
        dynamic_body_height_delay_counter=linearParameterChange(dynamic_body_height_delay_counter, dynamic_body_height_delay, 1);
        step_height_factor = linearParameterChange(step_height_factor,0,0.04);
    }else{
        dynamic_body_height_delay_counter=0;
        step_height_factor = linearParameterChange(step_height_factor,1,0.02);
    }
    step_height*=step_height_factor;
    if(dynamic_body_height_delay_counter>=dynamic_body_height_delay){
        dynamic_body_height = linearParameterChange(dynamic_body_height, body_height_stand, 0.0001f);
    } else {
        dynamic_body_height = linearParameterChange(dynamic_body_height, body_height, 0.0002f);
    }

    float swing_factor = parabolicReturn(min(1.f, t));
    float swing_height = - swing_factor* step_height;
    float support_height = - parabolicReturn(min(1.f, t * support_recover_factor + switch_phase)) * step_height;
    float odo_forward = -(support_foot > 0 ? dx_left : dx_right);
    float odo_left = -(support_foot > 0 ? dy_left : dy_right);
    float swing_spline_value = get_spline_value(min(t, 1.f), swing_spline);

    float dxTmp=dx;
    dx+=vx_correction_smooth*step_height_factor;
    update_velocities(swing_spline_value);
    dx=dxTmp;
    odo_forward += support_foot > 0 ? dx_left : dx_right;
    odo_left += support_foot > 0 ? dy_left : dy_right;
    odo->apply(-odo_forward, -odo_left, yaw);  // negate because support_foot moves backwards

    float hip_swing = 0.f;
    float side_left = -dy_left - 0.05f + (da_left - da_right) * 0.06f - fabs(da_left - da_right) * 0.03f +
                      hip_swing;  // TODO um Ball drehen fix
    float side_right = -dy_right + 0.05f - (da_left - da_right) * 0.06f + fabsf(da_left - da_right) * 0.03f + hip_swing;
    float support_left = support_foot > 0 ? support_height : swing_height;
    float support_right = support_foot > 0 ? swing_height : support_height;

    float waddle_left = step_height > 0 ? support_left / step_height * waddle_gain : 0;
    float waddle_right = step_height > 0 ? support_right / step_height * waddle_gain : 0;

    float body_shift_x_target = clamp(-max_deceleration_forward * step_duration, dx_request * step_duration - dx,
                                      max_acceleration_forward * step_duration) *
                                body_shift_amp + max(0.f,dy*da*body_shift_x_when_turn_factor);
    body_shift_x = body_shift_x * body_shift_smoothing + (1 - body_shift_smoothing) * body_shift_x_target;

    float body_shift_y = body_roll * (-0.032f + 0.09f * min(1.f, abs(dy) / 0.35f)) * body_shift_y_gyro_gain;
    float distance_x = dx_left - dx_right;
    float body_shift_x_correction=sin((t+body_shift_x_phase)*M_PI*2.f)*body_shift_x_gain*dx;
    float body_shift_y_correction=sin((t+body_shift_y_phase)*M_PI*2.f)*body_shift_y_gain*dy;

    kinematics::LegPositionPlain footL(true, dx_left + body_offset_x + body_shift_x + body_shift_x_correction, side_left + body_shift_y + body_shift_y_correction,
                                       support_left - distance_x * (stairway_gain+stairway_vx_correction), 0.05f, 0.f);
    kinematics::LegPositionPlain footR(false, dx_right + body_offset_x + body_shift_x + body_shift_x_correction, side_right + body_shift_y + body_shift_y_correction,
                                       support_right + distance_x * (stairway_gain+stairway_vx_correction), 0.05f, 0.f);
    kinematics::PositionPlain feet = kinematics::PositionPlain(footL, footR, da_right - da_left - v_angle);
    kinematics::Joints j = kinematics::InverseKinematics::setFeetRelToPlane(feet, dynamic_body_height);



    LegJoints result = set_joints(waddle_right, waddle_left, ankle_balancer, step_height, j);

    t += 1.f / (frames_per_second * (step_duration+step_duration_correction));
    return result;
}
