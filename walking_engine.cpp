#include "walking_engine.h"

#include <cmath>
#include <iostream>
#include <vector>

#include <InverseKinematics.h>
#include <LegPositionPlain.h>
#include <PositionPlain.h>
#include <joints.h>
// #include <sound_player.h>
#include <stl_ext.h>
// #include <visualizer.h>
#include "kalman_filter.h"

using namespace std;
using namespace htwk;

float WalkingEngine::step_duration = 0.255f;
float WalkingEngine::max_forward = 0.35f;
float WalkingEngine::max_backward = 0.25f;
float WalkingEngine::max_strafe = 0.38f;
float WalkingEngine::max_strafe_unsafe = 0.45f;
float WalkingEngine::max_turn = 1.7f;
float WalkingEngine::velocity_combination_damping = 1.f;
float WalkingEngine::max_acceleration_forward = 0.47f;  // m/s/s
float WalkingEngine::max_deceleration_forward = 0.235f;  // m/s/s
float WalkingEngine::max_acceleration_side = 0.49f;     // m/s/s
float WalkingEngine::max_acceleration_turn = 3.92f;      // rad/s/s

static void play_sound_file(const char * s) { /* please provide your own implementation */ }

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
    float left = fsr.left.fl + fsr.left.fr + fsr.left.rl + fsr.left.rr;
    float right = fsr.right.fl + fsr.right.fr + fsr.right.rl + fsr.right.rr;
    return (left - right) / (left + right);
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
        if (abs(dx) + abs(dy) + abs(da) < 0.001f){
            support_foot = dy_request > 0?-1:1;
        }else{
            support_foot = new_support_foot_prediction;
        }
        if (t > max_rel_step_duration * step_duration_factor &&
            abs(dx_request) + abs(dy_request) + abs(da_request) >= 0.001f) {
            if (step_duration_factor * max_rel_step_duration > 0.95f) {
                step_duration_factor *= 0.9f;
                support_foot = -support_foot;
            }
        } else {
            step_duration_factor = 1;
        }
        float accel_fac = std::min(1.f, t) * step_duration;
        t = 0;

        last_dx_left = dx_left;
        last_dx_right = dx_right;
        last_dy_left = dy_left;
        last_dy_right = dy_right;
        last_da_left = da_left;
        last_da_right = da_right;
        point_3d delta_req{dx_request * step_duration - dx,
                           dy_request * step_duration - dy,
                           da_request * step_duration - da};
        if (delta_req.norm() > 0) {
            point_3d delta_rel{delta_req.x / (delta_req.x > 0 ? max_acceleration_forward : max_deceleration_forward) / accel_fac,
                               delta_req.y / max_acceleration_side / accel_fac,
                               delta_req.z / max_acceleration_turn / accel_fac};
            float delta_fac = 1.f / std::max(1.f, delta_rel.norm());
            dx += delta_req.x * delta_fac;
            dy += delta_req.y * delta_fac;
            da += delta_req.z * delta_fac;
        }
        if(shoot_cooldown_count>0){
            shoot_cooldown_count-=1;
            shoot_active=0;
        }
        if(shoot_cooldown_count<=0&&shoot_request>0&&support_foot*shoot_request_side>0){
            shoot_cooldown_count=5;
            shoot_active=1;
            shoot_side=shoot_request_side;
            if(shoot_request_side>0){
                play_sound_file("shoot01.wav");
            }else{
                play_sound_file("shoot02.wav");
            }

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

LegJoints WalkingEngine::set_joints(float waddle_right, float waddle_left, float ankle_pitch_right ,float ankle_pitch_left, float ankle_roll_right ,float ankle_roll_left,  float extra_forward_right, float extra_forward_left, const AnkleBalancer& ankle_balancer,
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
        result[RAnkleRoll].angle += ankle_roll_right;
        result[LAnkleRoll].angle += ankle_roll_left;

        result[RAnklePitch].angle += -ankle_pitch_right + extra_forward_right;
        result[LAnklePitch].angle += -ankle_pitch_left + extra_forward_left;

        result[RKneePitch].angle -= extra_forward_right;
        result[LKneePitch].angle -= extra_forward_left;
    }
    set_stiffness(dynamic_body_height == body_height_stand ? 0.6f : 0.8f, &result);
    return result;
}

float WalkingEngine::deadband_filter(float min, float value, float max){
    if(value < min) {
        return value - min;
    }
    if(value > max) {
        return value - max;
    }
    return 0;
}

LegJoints WalkingEngine::proceed(const FSR& fsr, float body_pitch, float body_roll, const AnkleBalancer& ankle_balancer, float yaw,
                                 const LolaSensorFrame& sensor_frame, Odometry* odo, ArmController* arm_controller) {

    //body_pitch=body_pitch*body_pitch_damping+body_pitch_raw*(1-body_pitch_damping);
    float body_pitch_smooth = kalmanFilter.smooth(body_pitch);
    //body_pitch=body_pitch_smooth; //TODO: activate & test smooth body pitch for smoother walking
    arm_controller->request = ArmController::ArmRequest::BACK;

    float body_pitch_error=deadband_filter(body_pitch_deadband_min,body_pitch,body_pitch_deadband_max);//   body_pitch<0.15f?min(0.f,(body_pitch-0.05f)):max(0.f,body_pitch-0.25f);
    float gyro_pitch_error=deadband_filter(-gyro_deadband,ankle_balancer.gyroPitch,gyro_deadband);

    float step_duration_correction=clamp(-body_pitch_error*dx*step_duration_correction_gain,0.00f,0.1f);
    float vx_correction=clamp(body_pitch_error*vx_correction_bpitch_gain+gyro_pitch_error*vx_correction_gpitch_gain,vx_correction_min,vx_correction_max);
    float step_height_correction=0;
    if(shoot_active==0){
        vx_correction_smooth=linearParameterChange(vx_correction_smooth,vx_correction,vx_correction_step);
//        step_height_correction=clamp(abs(body_pitch_error*step_height_correction_gain),0.0f,step_height_correction_max);
    }else{
        vx_correction_smooth=linearParameterChange(vx_correction_smooth,0,vx_correction_step);
    }

    detect_and_proceed_phase_switch(fsr);

    float stairway_vx_correction=vx_correction_smooth*stairway_vx_correction_factor;

    float step_height = basic_step_height + forward_step_height * abs(dx) / step_duration+step_height_correction;

    if (abs(dx) + abs(dy) + abs(da) < 0.001f ) {
        //t = 0.6;
        dynamic_body_height_delay_counter=linearParameterChange(dynamic_body_height_delay_counter, dynamic_body_height_delay, 1);
        step_height_factor = linearParameterChange(step_height_factor,0,0.08);
    }else{
        dynamic_body_height_delay_counter=0;
        step_height_factor = linearParameterChange(step_height_factor,1,0.04);
    }
    //body_shift_x=clamp(body_shift_x+(fsr_ratio*(fsr.left.fl+fsr.left.fr+fsr.right.fl+fsr.right.fr)-(1-fsr_ratio)*(fsr.left.rl+fsr.left.rr+fsr.right.rl+fsr.right.rr))*dynamic_fsr_body_shift_factor/frames_per_second,-0.02f,0.01f);
    step_height*=step_height_factor;
    if(dynamic_body_height_delay_counter>=dynamic_body_height_delay){
        dynamic_body_height = linearParameterChange(dynamic_body_height, body_height_stand, 0.0001f);
    } else {
        dynamic_body_height = linearParameterChange(dynamic_body_height, body_height, 0.0002f);
    }

    float swing_factor = parabolicReturn(min(1.f, t));
    float swing_height = -swing_factor* step_height;
    float support_height = -parabolicReturn(min(1.f, t * support_recover_factor + switch_phase)) * step_height;
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
    float phase= tSmooth+(support_foot > 0?1:0);
    float hip_swing = (hip_swing_gain*step_height_factor+abs(dy)*hip_swing_side_gain)*sin(M_PI*phase+hip_swing_phase);
    float side_left = -dy_left - 0.05f + (da_left - da_right) * 0.06f - fabs(da_left - da_right) * 0.03f +
                      hip_swing;  // TODO um Ball drehen fix
    float side_right = -dy_right + 0.05f - (da_left - da_right) * 0.06f + fabsf(da_left - da_right) * 0.03f + hip_swing;
    float support_left = support_foot > 0 ? support_height : swing_height;
    float support_right = support_foot > 0 ? swing_height : support_height;

    float waddle_left = step_height > 0 ? support_left / step_height * waddle_gain : 0;
    float waddle_right = step_height > 0 ? support_right / step_height * waddle_gain : 0;

    float body_shift_x_target = stand_body_shift_x_offset*clamp(1.f-dx_request/max_forward,0.f,1.f)+clamp(dx_request * step_duration - dx,
                                      -max_deceleration_forward * step_duration,
                                      max_acceleration_forward * step_duration) *
                                body_shift_amp + min(0.f,dy*da*body_shift_x_when_turn_factor);
    body_shift_x = body_shift_x * body_shift_smoothing + (1 - body_shift_smoothing) * body_shift_x_target;


    //float body_shift_y = body_roll * (-0.032f + 0.09f * min(1.f, abs(dy) / 0.35f)) * body_shift_y_gyro_gain;
    float body_shift_y = body_roll*body_shift_y_roll;//body_roll * (-0.032f + 0.09f * min(1.f, abs(dy) / 0.35f)) * body_shift_y_gyro_gain;
    //float distance_x = dx_left - dx_right;
    float body_shift_x_correction=sin((tSmooth+body_shift_x_phase)*M_PI*2.f)*body_shift_x_gain*dx;
    float body_shift_y_correction=0;//sin((tSmooth+body_shift_y_phase)*M_PI*2.f)*body_shift_y_gain*dy;

    float shoot_factor = parabolicReturn(clamp((tSmooth+0.5f-shoot_phase)/max(0.05f,shoot_time), 0.f, 1.f));
    float shoot_x_left=shoot_active*(shoot_side<0?shoot_factor*shoot_shift_x*step_height_factor:shoot_factor*shoot_compensate_x*step_height_factor);
    float shoot_x_right=shoot_active*(shoot_side>0?shoot_factor*shoot_shift_x*step_height_factor:shoot_factor*shoot_compensate_x*step_height_factor);
    float shoot_ankle_pitch_left=shoot_active*(shoot_side<0?shoot_factor*shoot_pitch*step_height_factor:shoot_factor*shoot_compensate_pitch*step_height_factor);
    float shoot_ankle_pitch_right=shoot_active*(shoot_side>0?shoot_factor*shoot_pitch*step_height_factor:shoot_factor*shoot_compensate_pitch*step_height_factor);

    float extra_forward_left=shoot_active*(shoot_side<0?shoot_factor*shoot_knee*step_height_factor:0);
    float extra_forward_right=shoot_active*(shoot_side>0?shoot_factor*shoot_knee*step_height_factor:0);

    float shoot_start_factor = parabolicReturn(clamp((t+0.5f-shoot_start_phase)/max(0.05f,shoot_start_time), 0.f, 1.f));

    shoot_x_left+=shoot_active*(shoot_side<0?shoot_start_factor*shoot_start_shift_x*step_height_factor:0);
    shoot_x_right+=shoot_active*(shoot_side>0?shoot_start_factor*shoot_start_shift_x*step_height_factor:0);
    shoot_ankle_pitch_left+=shoot_active*(shoot_side<0?shoot_start_factor*shoot_start_pitch*step_height_factor:0);
    shoot_ankle_pitch_right+=shoot_active*(shoot_side>0?shoot_start_factor*shoot_start_pitch*step_height_factor:0);

    extra_forward_left+=shoot_active*(shoot_side<0?shoot_factor*shoot_start_knee*step_height_factor:0);
    extra_forward_right+=shoot_active*(shoot_side>0?shoot_factor*shoot_start_knee*step_height_factor:0);

    float shoot_ankle_roll_factor = parabolicReturn(clamp((t+0.5f-shoot_ankle_roll_phase)/max(0.05f,shoot_ankle_roll_time), 0.f, 1.f));
    float shoot_ankle_roll_left=shoot_active*(shoot_side<0?shoot_ankle_roll_factor*shoot_ankle_roll*step_height_factor:-swing_factor*shoot_support_ankle_roll*step_height_factor);
    float shoot_ankle_roll_right=shoot_active*(shoot_side>0?-shoot_ankle_roll_factor*shoot_ankle_roll*step_height_factor:swing_factor*shoot_support_ankle_roll*step_height_factor);

    float shoot_body_lean_y=shoot_side*shoot_active*shoot_body_shift_y*swing_factor;

    float footLX=dx_left + body_offset_x + body_shift_x + body_shift_x_correction + shoot_x_left;
    float footLY=side_left + body_shift_y + body_shift_y_correction + shoot_body_lean_y;
    float footLA=support_left - dx_left * (stairway_gain * sgn(dx) - stairway_vx_correction);

    float footRX=dx_right + body_offset_x + body_shift_x + body_shift_x_correction + shoot_x_right;
    float footRY=side_right + body_shift_y + body_shift_y_correction + shoot_body_lean_y;
    float footRA=support_right - dx_right * (stairway_gain * sgn(dx) - stairway_vx_correction);

    kinematics::LegPositionPlain footL{true, footLX, footLY,footLA, 0.05f, 0.f};
    kinematics::LegPositionPlain footR{false, footRX, footRY, footRA, 0.05f, 0.f};

    feet = {footL, footR, da_right - da_left - v_angle};
    float dx_circle=(dx_left-dx_right)*0;//*0.5;
    float circle_height=sqrt(circle_height_radius*circle_height_radius-dx_circle*dx_circle)-circle_height_radius;
    kinematics::Joints j = kinematics::InverseKinematics::setFeetRelToPlane(feet, dynamic_body_height+circle_height);
    num_frame++;
    // VisTransPtr imuStats = Visualizer::instance().startTransaction({}, "body", ABSOLUTE, NO_REPLACE);
    // imuStats->addParameter(Parameter::createFloat("pitch", body_pitch));
    // imuStats->addParameter(Parameter::createFloat("roll", body_roll));
    // imuStats->addParameter(Parameter::createInt("num_frame", num_frame));
    // imuStats->addParameter(Parameter::createFloat("footRX", footRX));
    // imuStats->addParameter(Parameter::createFloat("dx_right", dx_right));
    // imuStats->addParameter(Parameter::createFloat("dx", dx));
    // imuStats->addParameter(Parameter::createFloat("body_shift_x", body_shift_x));
    // imuStats->addParameter(Parameter::createFloat("body_shift_x_correction", body_shift_x_correction));
    // imuStats->addParameter(Parameter::createFloat("t", t));
    // imuStats->addParameter(Parameter::createFloat("tSmooth", tSmooth));
    // imuStats->addParameter(Parameter::createFloat("vx_correction_smooth", vx_correction_smooth));
    // imuStats->addParameter(Parameter::createFloat("body_pitch_smooth", body_pitch_smooth));

    // Visualizer::instance().commit(imuStats);


    LegJoints result = set_joints(waddle_right, waddle_left, shoot_ankle_pitch_right, shoot_ankle_pitch_left, shoot_ankle_roll_right, shoot_ankle_roll_left, extra_forward_right, extra_forward_left, ankle_balancer, step_height, j);

    float step_duration_frames=(frames_per_second * (step_duration+step_duration_correction+shoot_active*shoot_duration_increment));
    t+=1./step_duration_frames;
    if(tFull<=tClamp||tClamp==1||tFull>tClamp+0.5) {
        tFull+=1./step_duration_frames;
    }
    tClamp=min(t,1.f);
    if(tClamp<tFull-0.5) {
        if(tFull<=1) {
            tSmooth=tFull+tClamp-1;
        }else {
            tFull=max(tFull-1.f,tClamp);
            tSmooth=tClamp;
        }
    }else {
        tSmooth=tClamp;
    }
    if(tFull>1&&tClamp==1) {
        tSmooth=(tFull+tClamp)*0.5;
    }else if(tFull>tClamp&&tFull<0.5) {
        tSmooth=(tFull+tClamp)*0.5;
    }

    return result;
}
