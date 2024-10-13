#include <lola_frame.h>

#include <iostream>

#include <stl_ext.h>

LolaFrameHandler::LolaFrameHandler() {
    initSensorFrame();
    initActuatorFrame();
}

void LolaFrameHandler::initSensorFrame() {
    auto& gyr = sensor_frame.imu.gyr;
    auto& accel = sensor_frame.imu.accel;
    auto& head = sensor_frame.joints.head;
    auto& arms = sensor_frame.joints.arms;
    auto& legs = sensor_frame.joints.legs;
    auto& fsr = sensor_frame.fsr;
    auto& battery = sensor_frame.battery;
    auto& touch = sensor_frame.touch;

    sensor_frame_positions = {
            {"Accelerometer", {&accel.x, &accel.y, &accel.z}},
            {"Gyroscope", {&gyr.roll, &gyr.pitch, &gyr.yaw}},
            {"Battery", {&battery.charge, &battery.status, &battery.current, &battery.temp}},
            {"FSR",
             {&fsr.left.fl, &fsr.left.fr, &fsr.left.rl, &fsr.left.rr, &fsr.right.fl, &fsr.right.fr, &fsr.right.rl,
              &fsr.right.rr}},
            {"Position", {&head[HeadYaw].angle,        &head[HeadPitch].angle,     &arms[LShoulderPitch].angle,
                          &arms[LShoulderRoll].angle,  &arms[LElbowYaw].angle,     &arms[LElbowRoll].angle,
                          &arms[LWristYaw].angle,      &legs[HipYawPitch].angle,   &legs[LHipRoll].angle,
                          &legs[LHipPitch].angle,      &legs[LKneePitch].angle,    &legs[LAnklePitch].angle,
                          &legs[LAnkleRoll].angle,     &legs[RHipRoll].angle,      &legs[RHipPitch].angle,
                          &legs[RKneePitch].angle,     &legs[RAnklePitch].angle,   &legs[RAnkleRoll].angle,
                          &arms[RShoulderPitch].angle, &arms[RShoulderRoll].angle, &arms[RElbowYaw].angle,
                          &arms[RElbowRoll].angle,     &arms[RWristYaw].angle,     &arms[LHand].angle,
                          &arms[RHand].angle}},
            {"Stiffness",
             {&head[HeadYaw].stiffness,        &head[HeadPitch].stiffness,     &arms[LShoulderPitch].stiffness,
              &arms[LShoulderRoll].stiffness,  &arms[LElbowYaw].stiffness,     &arms[LElbowRoll].stiffness,
              &arms[LWristYaw].stiffness,      &legs[HipYawPitch].stiffness,   &legs[LHipRoll].stiffness,
              &legs[LHipPitch].stiffness,      &legs[LKneePitch].stiffness,    &legs[LAnklePitch].stiffness,
              &legs[LAnkleRoll].stiffness,     &legs[RHipRoll].stiffness,      &legs[RHipPitch].stiffness,
              &legs[RKneePitch].stiffness,     &legs[RAnklePitch].stiffness,   &legs[RAnkleRoll].stiffness,
              &arms[RShoulderPitch].stiffness, &arms[RShoulderRoll].stiffness, &arms[RElbowYaw].stiffness,
              &arms[RElbowRoll].stiffness,     &arms[RWristYaw].stiffness,     &arms[LHand].stiffness,
              &arms[RHand].stiffness}},
            {"Temperature",
             {&head[HeadYaw].temperature,        &head[HeadPitch].temperature,     &arms[LShoulderPitch].temperature,
              &arms[LShoulderRoll].temperature,  &arms[LElbowYaw].temperature,     &arms[LElbowRoll].temperature,
              &arms[LWristYaw].temperature,      &legs[HipYawPitch].temperature,   &legs[LHipRoll].temperature,
              &legs[LHipPitch].temperature,      &legs[LKneePitch].temperature,    &legs[LAnklePitch].temperature,
              &legs[LAnkleRoll].temperature,     &legs[RHipRoll].temperature,      &legs[RHipPitch].temperature,
              &legs[RKneePitch].temperature,     &legs[RAnklePitch].temperature,   &legs[RAnkleRoll].temperature,
              &arms[RShoulderPitch].temperature, &arms[RShoulderRoll].temperature, &arms[RElbowYaw].temperature,
              &arms[RElbowRoll].temperature,     &arms[RWristYaw].temperature,     &arms[LHand].temperature,
              &arms[RHand].temperature}},
            {"Current", {&head[HeadYaw].current,        &head[HeadPitch].current,     &arms[LShoulderPitch].current,
                         &arms[LShoulderRoll].current,  &arms[LElbowYaw].current,     &arms[LElbowRoll].current,
                         &arms[LWristYaw].current,      &legs[HipYawPitch].current,   &legs[LHipRoll].current,
                         &legs[LHipPitch].current,      &legs[LKneePitch].current,    &legs[LAnklePitch].current,
                         &legs[LAnkleRoll].current,     &legs[RHipRoll].current,      &legs[RHipPitch].current,
                         &legs[RKneePitch].current,     &legs[RAnklePitch].current,   &legs[RAnkleRoll].current,
                         &arms[RShoulderPitch].current, &arms[RShoulderRoll].current, &arms[RElbowYaw].current,
                         &arms[RElbowRoll].current,     &arms[RWristYaw].current,     &arms[LHand].current,
                         &arms[RHand].current}},
            {"Status", {(float*)&head[HeadYaw].status,        (float*)&head[HeadPitch].status,
                        (float*)&arms[LShoulderPitch].status, (float*)&arms[LShoulderRoll].status,
                        (float*)&arms[LElbowYaw].status,      (float*)&arms[LElbowRoll].status,
                        (float*)&arms[LWristYaw].status,      (float*)&legs[HipYawPitch].status,
                        (float*)&legs[LHipRoll].status,       (float*)&legs[LHipPitch].status,
                        (float*)&legs[LKneePitch].status,     (float*)&legs[LAnklePitch].status,
                        (float*)&legs[LAnkleRoll].status,     (float*)&legs[RHipRoll].status,
                        (float*)&legs[RHipPitch].status,      (float*)&legs[RKneePitch].status,
                        (float*)&legs[RAnklePitch].status,    (float*)&legs[RAnkleRoll].status,
                        (float*)&arms[RShoulderPitch].status, (float*)&arms[RShoulderRoll].status,
                        (float*)&arms[RElbowYaw].status,      (float*)&arms[RElbowRoll].status,
                        (float*)&arms[RWristYaw].status,      (float*)&arms[LHand].status,
                        (float*)&arms[RHand].status}},
            {"Touch",
             {&touch.chestButton, &touch.head.front, &touch.head.middle, &touch.head.rear, &touch.leftFoot.left,
              &touch.leftFoot.right, &touch.leftHand.back, &touch.leftHand.left, &touch.leftHand.right,
              &touch.rightFoot.left, &touch.rightFoot.right, &touch.rightHand.back, &touch.rightHand.left,
              &touch.rightHand.right}}};
}

void LolaFrameHandler::initActuatorFrame() {
    auto& head = actuator_frame.joints.head;
    auto& arms = actuator_frame.joints.arms;
    auto& legs = actuator_frame.joints.legs;
    auto& ears = actuator_frame.leds.ears;
    auto& eyes = actuator_frame.leds.eyes;
    auto& feet = actuator_frame.leds.feet;
    auto& chest = actuator_frame.leds.chest;
    auto& skull = actuator_frame.leds.skull;
    actuator_frame_positions = {
            {"LEar",
             {&ears.left[0], &ears.left[1], &ears.left[2], &ears.left[3], &ears.left[4], &ears.left[5], &ears.left[6],
              &ears.left[7], &ears.left[8], &ears.left[9]}},
            {"REar",
             {&ears.right[9], &ears.right[8], &ears.right[7], &ears.right[6], &ears.right[5], &ears.right[4],
              &ears.right[3], &ears.right[2], &ears.right[1], &ears.right[0]}},
            {"LEye",
             {&eyes.left[1].r, &eyes.left[0].r, &eyes.left[7].r, &eyes.left[6].r, &eyes.left[5].r, &eyes.left[4].r,
              &eyes.left[3].r, &eyes.left[2].r, &eyes.left[1].g, &eyes.left[0].g, &eyes.left[7].g, &eyes.left[6].g,
              &eyes.left[5].g, &eyes.left[4].g, &eyes.left[3].g, &eyes.left[2].g, &eyes.left[1].b, &eyes.left[0].b,
              &eyes.left[7].b, &eyes.left[6].b, &eyes.left[5].b, &eyes.left[4].b, &eyes.left[3].b, &eyes.left[2].b}},
            {"REye", {&eyes.right[0].r, &eyes.right[1].r, &eyes.right[2].r, &eyes.right[3].r, &eyes.right[4].r,
                      &eyes.right[5].r, &eyes.right[6].r, &eyes.right[7].r, &eyes.right[0].g, &eyes.right[1].g,
                      &eyes.right[2].g, &eyes.right[3].g, &eyes.right[4].g, &eyes.right[5].g, &eyes.right[6].g,
                      &eyes.right[7].g, &eyes.right[0].b, &eyes.right[1].b, &eyes.right[2].b, &eyes.right[3].b,
                      &eyes.right[4].b, &eyes.right[5].b, &eyes.right[6].b, &eyes.right[7].b}},
            {"LFoot", {&feet.left.r, &feet.left.g, &feet.left.b}},
            {"RFoot", {&feet.right.r, &feet.right.g, &feet.right.b}},
            {"Chest", {&chest.r, &chest.g, &chest.b}},
            {"Skull", {&skull[0], &skull[1], &skull[2], &skull[3], &skull[4], &skull[5], &skull[6], &skull[7],
                       &skull[8], &skull[9], &skull[10], &skull[11]}},
            {"Position", {&head[HeadYaw].angle,        &head[HeadPitch].angle,     &arms[LShoulderPitch].angle,
                          &arms[LShoulderRoll].angle,  &arms[LElbowYaw].angle,     &arms[LElbowRoll].angle,
                          &arms[LWristYaw].angle,      &legs[HipYawPitch].angle,   &legs[LHipRoll].angle,
                          &legs[LHipPitch].angle,      &legs[LKneePitch].angle,    &legs[LAnklePitch].angle,
                          &legs[LAnkleRoll].angle,     &legs[RHipRoll].angle,      &legs[RHipPitch].angle,
                          &legs[RKneePitch].angle,     &legs[RAnklePitch].angle,   &legs[RAnkleRoll].angle,
                          &arms[RShoulderPitch].angle, &arms[RShoulderRoll].angle, &arms[RElbowYaw].angle,
                          &arms[RElbowRoll].angle,     &arms[RWristYaw].angle,     &arms[LHand].angle,
                          &arms[RHand].angle}},
            {"Stiffness",
             {&head[HeadYaw].stiffness,        &head[HeadPitch].stiffness,     &arms[LShoulderPitch].stiffness,
              &arms[LShoulderRoll].stiffness,  &arms[LElbowYaw].stiffness,     &arms[LElbowRoll].stiffness,
              &arms[LWristYaw].stiffness,      &legs[HipYawPitch].stiffness,   &legs[LHipRoll].stiffness,
              &legs[LHipPitch].stiffness,      &legs[LKneePitch].stiffness,    &legs[LAnklePitch].stiffness,
              &legs[LAnkleRoll].stiffness,     &legs[RHipRoll].stiffness,      &legs[RHipPitch].stiffness,
              &legs[RKneePitch].stiffness,     &legs[RAnklePitch].stiffness,   &legs[RAnkleRoll].stiffness,
              &arms[RShoulderPitch].stiffness, &arms[RShoulderRoll].stiffness, &arms[RElbowYaw].stiffness,
              &arms[RElbowRoll].stiffness,     &arms[RWristYaw].stiffness,     &arms[LHand].stiffness,
              &arms[RHand].stiffness}}};
}

LolaSensorFrame& LolaFrameHandler::unpack(const char* buffer, size_t size) {
    msgpack::object_handle oh = msgpack::unpack(buffer, size);
    if (oh->is_nil()) {
        std::cerr << "No MsgPack message in LoLA message." << std::endl;
        return sensor_frame;
    }
    const auto& map = oh.get().via.map;
    auto* category = map.ptr;

    for (uint32_t i = 0; i < map.size; i++, category++) {
        if (auto ptr = find(sensor_frame_positions, category->key.as<std::string>())) {
            zip(*ptr, category->val.as<std::vector<float>>(), [](float* r, float f) {
                if (r != nullptr)
                    *r = f;
            });
        }
    }
#ifndef WEBOTS
    if (old_imu.gyr.yaw == sensor_frame.imu.gyr.yaw && old_imu.gyr.pitch == sensor_frame.imu.gyr.pitch &&
        old_imu.gyr.roll == sensor_frame.imu.gyr.roll)
        gyro_age++;
    else
        gyro_age = 0;
    if (old_imu.accel.x == sensor_frame.imu.accel.x && old_imu.accel.y == sensor_frame.imu.accel.y &&
        old_imu.accel.z == sensor_frame.imu.accel.z)
        accel_age++;
    else
        accel_age = 0;
#endif
    old_imu = sensor_frame.imu;
    return sensor_frame;
}

std::pair<char*, size_t> LolaFrameHandler::pack() {
    buffer.clear();
    msgpack::packer<msgpack::sbuffer> pk(&buffer);
    pk.pack_map(actuator_frame_positions.size());
    for (const auto& [name, ptrs] : actuator_frame_positions) {
        pk.pack(name);
        pk.pack_array(ptrs.size());
        for (float* val : ptrs) {
            pk.pack_fix_float(*val);
        }
    }
    return {buffer.data(), buffer.size()};
}
