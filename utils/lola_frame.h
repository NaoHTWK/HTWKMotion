#pragma once

#include <map>
#include <string>
#include <vector>

#include <msgpack.hpp>

#include <battery.h>
#include <fsr.h>
#include <imu.h>
#include <joints.h>
#include <leds.h>
#include <touch_sensors.h>

struct LolaSensorFrame {
    Joints joints;
    IMU imu;
    FSR fsr;
    Battery battery;
    TouchSensors touch;
};

struct LolaActuatorFrame {
    Joints joints;
    Leds leds;
};

class LolaFrameHandler {
public:
    LolaFrameHandler();
    LolaSensorFrame& unpack(const char* buffer, size_t size);
    std::pair<char*, size_t> pack();
    bool imuStuck() {
        return gyro_age > 5 || accel_age > 5;
    }

    LolaActuatorFrame actuator_frame;

private:
    void initSensorFrame();
    void initActuatorFrame();

    LolaSensorFrame sensor_frame;
    std::map<std::string, std::vector<float*>> sensor_frame_positions;
    std::map<std::string, std::vector<float*>> actuator_frame_positions;
    msgpack::sbuffer buffer;
    int gyro_age = 0;
    int accel_age = 0;
    IMU old_imu;
};
