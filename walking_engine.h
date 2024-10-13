#pragma once

#include <cmath>

#include <../kinematics/Joints.h>
#include <PositionPlain.h>
#include <ankle_balancer.h>
#include <arm_controller.h>
#include <fsr.h>
#include <joints.h>
#include <lola_frame.h>
#include <odometry.h>
#include <point_3d.h>
#include <robotoption.h>
#include "kalman_filter.h"

using namespace htwk;
using namespace NaoControl;
using namespace std;

#define NUM_SPLINE_POINTS 5

class WalkingEngine {

public:
    float frames_per_second = 1.f / 0.012f;
    static float step_duration;             // s
    static float max_forward;                // m/s
    static float max_backward;               // m/s
    static float max_strafe;                 // m/s
    static float max_strafe_unsafe;          // m/s
    static float max_turn;                   // rad/s
    static float max_acceleration_forward;  // m/s/s
    static float max_deceleration_forward;  // m/s/s
    static float max_acceleration_side;     // m/s/s
    static float max_acceleration_side_unsafe;     // m/s/s
    static float max_acceleration_turn;      // rad/s/s
    float max_acceleration_v_angle = 0.125f;   // rad/s/s
    float min_rel_step_duration = 0.75f;
    float max_rel_step_duration = 1.7;
    float step_duration_factor = 1;
    float v_angle = 0;
    float body_height = 0.19f;
    float body_height_stand = 0.201f;
    int dynamic_body_height_delay = 400;
    float body_shift_amp = -0.3f;
    float body_shift_smoothing = 0.98f;
    float body_shift_x = -0.01;
    float fsr_ratio=0.3;
    float dynamic_fsr_body_shift_factor=0.0005;
    float hip_swing_gain=0;
    float hip_swing_phase=-1;
    float hip_swing_side_gain=0.08;

    float waddle_gain = 0.0f;
    float stairway_gain = 0.005f;
    float basic_step_height = 0.0135f;
    float forward_step_height = 0.0145f;
    float support_recover_factor = 0.3334f;
    static float velocity_combination_damping;
    float body_offset_x = -0.005f;
    float circle_height_radius=0.1;
    float support_foot_swap_factor = 3;
    float body_shift_y_gyro_gain = 1;
    float swing_spline[NUM_SPLINE_POINTS * NUM_SPLINE_POINTS]{0, -0.177f, 0.273f, 1.11f, 1.};

    float body_shift_x_phase=0.4;
    float body_shift_x_gain=0.1;
    float body_shift_y_phase=-0.19;
    float body_shift_y_gain=0.047;
    float body_shift_y_roll=-0.05;

    float footL1Old[3]{0, 0,0};
    float footR1Old[3]{0, 0,0};
    float footL2Old[3]{0, 0,0};
    float footR2Old[3]{0, 0,0};
    float footL3Old[3]{0, 0,0};
    float footR3Old[3]{0, 0,0};

    float dynamic_body_height = body_height_stand;
    int dynamic_body_height_delay_counter = dynamic_body_height_delay;
    float step_height_factor = 0;
    float vx_correction_speed=0.01;

    float shoot_request=0;
    float shoot_request_side=1;
    float shoot_active=0;
    float shoot_cooldown_count=0;
    float shoot_side=1;

    float shoot_duration_increment=0.21250000316649675;

    float shoot_body_shift_y=0.01093750016298145;
    float shoot_support_ankle_roll=-0.19375000288709998;

    float shoot_ankle_roll_phase=0.484375;
    float shoot_ankle_roll_time=0.359375;
    float shoot_ankle_roll=-0.390625;

    float shoot_phase=0.65625;
    float shoot_time=0.59375;
    float shoot_shift_x=0.08906250353902578;
    float shoot_pitch=0.34375;
    float shoot_knee=-0.07500000111758709;
    float shoot_compensate_x=-0.012500000186264515;
    float shoot_compensate_pitch=-0.15;

    float shoot_start_phase=0.25;
    float shoot_start_time=0.28125;
    float shoot_start_shift_x=-0.02500000037252903;
    float shoot_start_pitch=-0.3125;
    float shoot_start_knee=0.25;

    float vx_correction_bpitch_gain=0.25;
    float vx_correction_gpitch_gain=0.0;
    float step_duration_correction_gain=0;
    float vx_correction_gain=0.0;
    float step_height_correction_gain=0.2;
    float step_height_correction_max=0.008;
    float gyro_deadband=0;
    float body_pitch_deadband_min=0.10;
    float body_pitch_deadband_max=0.25;
    float vx_correction_min=-0.305;
    float vx_correction_max=0.25;
    float vx_correction_smooth=0;
    float vx_correction_step=0.005;
    float stairway_vx_correction_factor=0.5;
    float body_shift_x_when_turn_factor=0.1;
    float stand_body_shift_x_offset=-0.006;


    kinematics::PositionPlain feet;
    SmoothingFilter kalmanFilter;

    WalkingEngine() : kalmanFilter(5.f) {
        auto* headOptions = new NaoControl::OptionSet("walkingengine");
//        headOptions->addOption(new FloatOption("body_height", &(body_height), 0.14f, 0.2f, 0.005f));
        headOptions->addOption(new FloatOption("bpitch_filter_window_size", &kalmanFilter.window_size, 3.f, 100.f, 1.f));

        headOptions->addOption(new FloatOption("body_shift_x_phase", &(body_shift_x_phase), -0.5f, 0.5f, 0.005f));
        headOptions->addOption(new FloatOption("body_shift_x_gain", &(body_shift_x_gain), -0.25f, 0.25f, 0.005f));
        headOptions->addOption(new FloatOption("body_shift_y_phase", &(body_shift_y_phase), -0.19f, -0.5f, 0.005f));
        headOptions->addOption(new FloatOption("body_shift_y_gain", &(body_shift_y_gain), 0.047f, 0.25f, 0.005f));
        headOptions->addOption(new FloatOption("stand_body_shift_x_offset", &(stand_body_shift_x_offset), 0.047f, 0.25f, 0.005f));

        headOptions->addOption(new FloatOption("min_rel_step_duration", &(min_rel_step_duration), 0.047f, 0.25f, 0.005f));
        headOptions->addOption(new FloatOption("max_rel_step_duration", &(max_rel_step_duration), 0.047f, 0.25f, 0.005f));
        headOptions->addOption(new FloatOption("spline_a", &(swing_spline[1]), 0.047f, 0.25f, 0.005f));
        headOptions->addOption(new FloatOption("spline_b", &(swing_spline[2]), 0.047f, 0.25f, 0.005f));
        headOptions->addOption(new FloatOption("spline_c", &(swing_spline[3]), 0.047f, 0.25f, 0.005f));


//        headOptions->addOption(new FloatOption("step_duration", &(step_duration), 0.25f, 0.28f, 0.005f));
        headOptions->addOption(new FloatOption("stairway_gain", &(stairway_gain), 0.0f, 0.03f, 0.005f));
        headOptions->addOption(new FloatOption("waddle_gain", &(waddle_gain), 0.0f, 0.03f, 0.005f));

        headOptions->addOption(new FloatOption("fsr_ratio", &(fsr_ratio), 0.0f, 1.0f, 0.1f));
        headOptions->addOption(new FloatOption("body_shift_smoothing", &(body_shift_smoothing), 0.0f, 1.0f, 0.1f));
        headOptions->addOption(new FloatOption("forward_step_height", &(forward_step_height), 0.0f, 1.0f, 0.1f));
        headOptions->addOption(new FloatOption("support_recover_factor", &(support_recover_factor), 0.0f, 1.0f, 0.1f));
        headOptions->addOption(new FloatOption("body_shift_amp", &(body_shift_amp), 0.0f, 1.0f, 0.1f));
        headOptions->addOption(new FloatOption("dynamic_fsr_body_shift_factor", &(dynamic_fsr_body_shift_factor), -0.5f, 0.5f, 0.0001f));
        headOptions->addOption(new FloatOption("max_forward", &(max_forward), 0.2f, 0.34f, 0.005f));
        headOptions->addOption(new FloatOption("max_backward", &(max_backward), 0.3f, 0.38f, 0.005f));
        headOptions->addOption(new FloatOption("step_duration", &(step_duration), 0.24f, 0.3f, 0.002f));
        headOptions->addOption(new FloatOption("basic_step_height", &(basic_step_height), 0.0135f, 0.013f, 0.014f));
        headOptions->addOption(new FloatOption("body_offset_x", &(body_offset_x), -0.1f, 0.1f, 0.005f));
        headOptions->addOption(new FloatOption("circle_height_radius", &(circle_height_radius), -0.1f, 0.1f, 0.005f));
        headOptions->addOption(new FloatOption("vx_correction_bpitch_gain", &(vx_correction_bpitch_gain), -0.1f, 0.1f, 0.005f));
        headOptions->addOption(new FloatOption("vx_correction_gpitch_gain", &(vx_correction_gpitch_gain), -0.1f, 0.1f, 0.005f));
        headOptions->addOption(new FloatOption("step_duration_correction_gain", &(step_duration_correction_gain), 0.0f, 5.f, 0.005f));
        headOptions->addOption(new FloatOption("vx_correction_gain", &(vx_correction_gain), -10.0f, 10.f, 0.005f));
        headOptions->addOption(new FloatOption("step_height_correction_gain", &(step_height_correction_gain), -10.0f, 10.f, 0.005f));
        headOptions->addOption(new FloatOption("step_height_correction_max", &(step_height_correction_max), 0.0f, 0.1f, 0.005f));
        headOptions->addOption(new FloatOption("vx_correction_speed", &(vx_correction_speed), -0.1f, 0.1f, 0.005f));
        headOptions->addOption(new FloatOption("gyro_deadband", &(gyro_deadband), -1.0f, 1.0f, 0.005f));
        headOptions->addOption(new FloatOption("body_pitch_deadband_min", &(body_pitch_deadband_min), -0.2f, 0.15f, 0.005f));
        headOptions->addOption(new FloatOption("body_pitch_deadband_max", &(body_pitch_deadband_max), 0.15f, 0.4f, 0.005f));
        headOptions->addOption(new FloatOption("vx_correction_min", &(vx_correction_min), -0.4f, 0, 0.005f));
        headOptions->addOption(new FloatOption("vx_correction_max", &(vx_correction_max), 0.0f, 0.4f, 0.005f));
        headOptions->addOption(new FloatOption("vx_correction_step", &(vx_correction_step), 0.0f, 0.25f, 0.005f));
        headOptions->addOption(new FloatOption("stairway_vx_correction_factor", &(stairway_vx_correction_factor), 0.0f, 0.25f, 0.005f));
        headOptions->addOption(new FloatOption("body_shift_x_when_turn_factor", &(body_shift_x_when_turn_factor), -0.05f, 0.05f, 0.005f));
        headOptions->addOption(new FloatOption("hip_swing_gain", &(hip_swing_gain), 3.0f, 1.0f, 0.1f));
        headOptions->addOption(new FloatOption("hip_swing_phase", &(hip_swing_phase), 1.0f, 1.0f, 0.1f));
        headOptions->addOption(new FloatOption("hip_swing_side_gain", &(hip_swing_side_gain), 1.0f, 1.0f, 0.1f));


        headOptions->addOption(new FloatOption("support_foot_swap_factor", &(support_foot_swap_factor), 3.0f, 1.0f, 0.1f));
        headOptions->addOption(new FloatOption("body_shift_y_gyro_gain", &(body_shift_y_gyro_gain), 1.0f, 1.0f, 0.1f));

        headOptions->addOption(new FloatOption("shoot_request", &(shoot_request), 0.0f, 1.0f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_request_side", &(shoot_request_side), 0.0f, 1.0f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_duration_increment", &(shoot_duration_increment), 0.0f, 0.8f, 0.005f));

        headOptions->addOption(new FloatOption("shoot_body_shift_y", &(shoot_body_shift_y), -0.05f, 0.05f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_support_ankle_roll", &(shoot_support_ankle_roll), -0.2f, 0.2f, 0.005f));

        headOptions->addOption(new FloatOption("shoot_ankle_roll_phase", &(shoot_ankle_roll_phase), 0.0f, 1.0f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_ankle_roll_time", &(shoot_ankle_roll_time), 0.0f, 1.0f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_ankle_roll", &(shoot_ankle_roll), -0.5f, 0.5f, 0.005f));

        headOptions->addOption(new FloatOption("shoot_phase", &(shoot_phase), 0.0f, 1.0f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_time", &(shoot_time), 0.0f, 1.0f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_shift_x", &(shoot_shift_x), 0.0f, 0.15f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_pitch", &(shoot_pitch), -0.5f, 0.5f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_knee", &(shoot_knee), -0.4f, 0.4f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_compensate_x", &(shoot_compensate_x), -0.05f, 0.05f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_compensate_pitch", &(shoot_compensate_pitch), -0.2f, 0.2f, 0.005f));


        headOptions->addOption(new FloatOption("shoot_start_shift_x", &(shoot_start_shift_x), -0.1f, 0.0f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_start_pitch", &(shoot_start_pitch), -0.5f, 0.5f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_start_knee", &(shoot_start_knee), -0.5f, 0.5f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_start_phase", &(shoot_start_phase), 0.0f, 1.0f, 0.005f));
        headOptions->addOption(new FloatOption("shoot_start_time", &(shoot_start_time), 0.0f, 1.0f, 0.005f));

        NaoControl::RobotOption::instance().addOptionSet(headOptions);


    }

    void reset() {
        support_foot = 0;
        switch_phase = 0;
        t = 0;
        dx_left = 0.f;
        dx_right = 0.f;
        last_dx_left = 0.f;
        last_dx_right = 0.f;
        dy_left = 0.f;
        dy_right = 0.f;
        last_dy_left = 0.f;
        last_dy_right = 0.f;
        da_left = 0.f;
        da_right = 0.f;
        last_da_left = 0.f;
        last_da_right = 0.f;
        dx = 0.f;  // m/step
        dy = 0.f;
        da = 0.f;
    }
    bool isStanding() const {
        return dx * dx + dy * dy + da * da == 0;
    }
    LegJoints proceed(const FSR& fsr, float body_pitch, float body_roll, const AnkleBalancer& ankle_balancer, float yaw,
                      const LolaSensorFrame &sensor_frame, Odometry* odo, ArmController* arm_controller);
    void setRequest(float dx, float dy, float da, float foot_v_angle, Shoot shoot, bool unsafe) {
        normSpeed(&dx, &dy, &da, unsafe);
        dx_request = dx;
        dy_request = dy;
        da_request = da;
        v_angle_request = foot_v_angle;
        if(shoot==Shoot::LEFT){
            shoot_request=1;
            shoot_request_side=-1;
        }
        if(shoot==Shoot::RIGHT){
            shoot_request=1;
            shoot_request_side=1;
        }
        if(shoot==Shoot::NONE){
            shoot_request=0;
        }
    }

    float get_walk_phase() {
        return t*support_foot;
    }

    point_3d walkSpeed() const {
        return {dx * step_duration, dy * step_duration, da * step_duration};
    }

    // For short distances up to 1m.
    static float walkingTime(float dx, float dy, float da) {
        return 1.f / normFactor(dx, dy, da, false);
    }

    static float normFactor(float dx, float dy, float da, bool unsafe) {
        point_3d p{dx / (dx >= 0.f ? max_forward : max_backward), dy / (unsafe ? max_strafe_unsafe : max_strafe), da / max_turn};
        if (p.x == 0 && p.y == 0 && p.z == 0)
            return 1;
        float norm_factor_ellipse = 1.f / p.norm();
        // float norm_factor_manhattan = 1.f / std::max(std::abs(p.x), std::max(std::abs(p.y), std::abs(p.z)));
        return norm_factor_ellipse;
    }
    static void normSpeed(float* dx, float* dy, float* da, bool unsafe) {
        float norm_factor = normFactor(*dx, *dy, *da, unsafe);
        if (norm_factor < 1.f) {
            *dx *= norm_factor;
            *dy *= norm_factor;
            *da *= norm_factor;
        }
    }

private:
    float calcSupportFoot(FSR fsr);
    float get_spline_value(float t, float* swing_spline);
    float dx_request = 0.f;  // m/s
    float dy_request = 0.f;
    float da_request = 0.f;
    float v_angle_request = 0.f;
    EMA gyroRoll{0.f, 0.7f};
    float roll = 0;
    FSR fsr_max_values;
    float support_foot = 0;
    float switch_phase = 0;
    float t = 0;
    int num_frame = 0;
    float tFull=0;
    float tClamp=0;
    float tSmooth=0;
    float dx_left = 0.f;
    float dx_right = 0.f;
    float last_dx_left = 0.f;
    float last_dx_right = 0.f;
    float dy_left = 0.f;
    float dy_right = 0.f;
    float last_dy_left = 0.f;
    float last_dy_right = 0.f;
    float da_left = 0.f;
    float da_right = 0.f;
    float last_da_left = 0.f;
    float last_da_right = 0.f;
    float dx = 0.f;  // m/step
    float dy = 0.f;
    float da = 0.f;
    float arm_state = -1.f;
    float last_support_foot_value = 0.f;
    void proceedWalkMotion();
    void detect_and_proceed_phase_switch(const FSR& fsr);
    float deadband_filter(float min, float value, float max);
    float linearParameterChange(float in, float target, float delta);
    void update_velocities(float swing_spline_value);
    LegJoints set_joints(float waddle_right, float waddle_left, float shoot_ankle_pitch_right, float shoot_ankle_pitch_left, float ankle_roll_right ,float ankle_roll_left, float extra_forward_right, float extra_forward_left, const AnkleBalancer& ankle_balancer, float step_height,
                         const kinematics::Joints& j);
};
