#ifndef TOUCH_SENSORS_H
#define TOUCH_SENSORS_H

struct TochSensorHand {
    float back{};
    float left{};
    float right{};
};

struct TochSensorFoot {
    float left{};
    float right{};
};

struct TochSensorHead {
    float front{};
    float middle{};
    float rear{};
};

struct TouchSensors {
    TochSensorHand leftHand;
    TochSensorHand rightHand;
    TochSensorFoot leftFoot;
    TochSensorFoot rightFoot;
    TochSensorHead head;
    float chestButton{};
};


#endif // TOUCH_SENSORS_H
