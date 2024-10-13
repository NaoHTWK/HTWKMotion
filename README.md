# HTWKMotion

Walking engine of the [HTWK Robots](https://htwk.bot/) Robot Soccer Team participating in [RoboCup SPL](https://spl.robocup.org/).

## INSTALLATION

The walking engine is designed to be used with our [LolaConnector](https://github.com/NaoHTWK/LolaConnector).
It also includes some auxiliary libraries for moving the arms and sitting down.

Integration of the walking engine in LolaConnector could look like this:
```c++
if (client_connected && !lola_shutdown && !lola_sit_forever) {
    if (!sit_motion.isStanding()) {
        joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
        joints.arms = arm_controller.proceed();
        walking_engine.reset();
        // TODO: You probably want to have a few else if statements here for things that should override the walking, e.g. getting up.
    } else {
        joints.legs = walking_engine.proceed(sensor_frame.fsr, imu_filter.angles.pitch, imu_filter.angles.roll, ankle_balancer,
                                             imu_filter.angles.yaw, &odo, &arm_controller);
        joints.arms = arm_controller.proceed();
    }
} else {
    if (walking_engine.isStanding()) {
        joints.legs = sit_motion.sitDown(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
    } else {
        walking_engine.setRequest(0, 0, 0, 0, Shoot::NONE);
        joints.legs = walking_engine.proceed(sensor_frame.fsr, imu_filter.angles.pitch, imu_filter.angles.roll, ankle_balancer,
                                                imu_filter.gyr.yaw, &odo, &arm_controller);
    }
    joints.arms = arm_controller.proceed();
}
```

To make the robot walk just call `walking_engine.setRequest()`.

To build:
```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../YOUR-cross-config.cmake
cmake --build .
```

Usage of a custom `cross-config.cmake` is optinal. An example `cross-config.cmake` can be found in our LolaConnector project.

The website for the [Eigen](http://eigen.tuxfamily.org) library seems down, so we include a copy of the library in the `3rdparty/eigen` folder. We also ship a msgpack library in `3rdparty/msgpack` for simplicty of building the project. Feel free to substitute your own libraries if desired.

All 3rd party code retains their respective license. The following license only applies to code from team HTWK Robots (formerly Nao-Team HTWK).

## LICENSE

Copyright (c) 2024 HTWK Robots. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.

3. The end-user documentation included with the redistribution, if
   any, must include the following acknowledgment:
   _"This product includes software developed by Team HTWK Robots
   ([htwk-robots.de](http://www.htwk-robots.de))."_
   Alternately, this acknowledgment may appear in the software
   itself, if and wherever such third-party acknowledgments
   normally appear.

4. For each HTWK Robots code release from which parts are used in
   a RoboCup competition, the usage shall be announced in the SPL
   mailing list (currently robocup-nao@cc.gatech.edu) one month
   before the first competition in which you are using it. The
   announcement shall name which parts of this code are used.

5. Bug fixes regarding existing code shall be sent back to
   HTWK Robots via GitHub pull requests
   (https://github.com/NaoHTWK).

THIS SOFTWARE IS PROVIDED BY HTWK Robots "AS IS" AND ANY
EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
NAO-TEAM HTWK NOR ITS MEMBERS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

