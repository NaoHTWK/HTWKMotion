#pragma once

#include <cmath>

#include <Joints.h>
#include <ankle_balancer.h>
#include <arm_controller.h>
#include <fsr.h>
#include <joints.h>
#include <odometry.h>
#include <point_3d.h>

using namespace htwk;
using namespace std;

#define NUM_SPLINE_POINTS 5

class WalkingEngine {

public:
    float frames_per_second = 1.f / 0.012f;
    float step_duration = 0.27f;             // s
    static float max_forward;                // m/s
    static float max_backward;               // m/s
    static float max_strafe;                 // m/s
    static float max_turn;                   // rad/s
    float max_acceleration_forward = 0.14f;  // m/s/s
    float max_deceleration_forward = 0.16f;  // m/s/s
    float max_acceleration_side = 0.12f;     // m/s/s
    float max_acceleration_turn = 1.2f;      // rad/s/s
    float max_acceleration_v_angle = 0.5f;   // rad/s/s
    float min_rel_step_duration = 0.75f;
    float max_rel_step_duration = 3;
    float step_duration_factor = 1;
    float v_angle = 0;
    float body_height = 0.19f;
    float body_height_stand = 0.201f;
    int dynamic_body_height_delay = 400;
    float body_shift_amp = -0.08f;
    float body_shift_smoothing = 0.95f;
    float body_shift_x = 0;

    float waddle_gain = 0.0f;
    float stairway_gain = 0.0075f;
    float basic_step_height = 0.0135f;
    float forward_step_height = 0.0142f;
    float support_recover_factor = 0.3334f;
    static float velocity_combination_damping;
    float body_offset_x = -0.01f;
    float support_foot_swap_factor = 3;
    float body_shift_y_gyro_gain = 1;
    float swing_spline[NUM_SPLINE_POINTS * NUM_SPLINE_POINTS]{0, -0.177f, 0.273f, 1.11f, 1.};
    float body_shift_x_phase=0.4;
    float body_shift_x_gain=0.1;
    float body_shift_y_phase=-0.19;
    float body_shift_y_gain=0.047;



    float dynamic_body_height = body_height_stand;
    int dynamic_body_height_delay_counter = dynamic_body_height_delay;
    float step_height_factor = 0;
    float vx_correction_speed=0.01;

    float vx_correction_bpitch_gain=0.15;
    float vx_correction_gpitch_gain=0.0;
    float step_duration_correction_gain=0;
    float vx_correction_gain=0.0;
    float step_height_correction_gain=0.2;
    float step_height_correction_max=0.008;
    float gyro_deadband=0;
    float body_pitch_deadband_min=0.10;
    float body_pitch_deadband_max=0.25;
    float vx_correction_min=-0.25;
    float vx_correction_max=0.25;
    float vx_correction_smooth=0;
    float vx_correction_step=0.005;
    float stairway_vx_correction_factor=0.2;
    float body_shift_x_when_turn_factor=0.15;

    WalkingEngine() {
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
    LegJoints proceed(const FSR& fsr, float body_pitch, float body_roll, const AnkleBalancer& ankle_balancer, float yaw, Odometry* odo,
                      ArmController* arm_controller);
    void setRequest(float dx, float dy, float da, float foot_v_angle) {
        normSpeed(&dx, &dy, &da);
        dx_request = dx;
        dy_request = dy;
        da_request = da;
        v_angle_request = foot_v_angle;
    }
    // For short distances up to 1m.
    static float walkingTime(float dx, float dy, float da) {
        return 1.f / normFactor(dx, dy, da);
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
    LegJoints set_joints(float waddle_right, float waddle_left, const AnkleBalancer& ankle_balancer, float step_height,
                         const kinematics::Joints& j);
    static float normFactor(float dx, float dy, float da) {
        point_3d p{dx / (dx >= 0.f ? max_forward : max_backward), dy / max_strafe, da / max_turn};
        if (p.norm_sqr() == 0)
            return 1;
        float norm_factor_ellipse = 1.f / p.norm();
        float norm_factor_manhattan = 1.f / std::max(std::abs(p.x), std::max(std::abs(p.y), std::abs(p.z)));
        return norm_factor_ellipse * velocity_combination_damping +
               norm_factor_manhattan * (1 - velocity_combination_damping);
    }
    void normSpeed(float* dx, float* dy, float* da) {
        float norm_factor = normFactor(*dx, *dy, *da);
        if (norm_factor < 1.f) {
            *dx *= norm_factor;
            *dy *= norm_factor;
            *da *= norm_factor;
        }
    }
};
