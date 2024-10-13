cmake_minimum_required(VERSION 3.24)

project(WalkingEngine CXX)

add_subdirectory(3rdparty/eigen)
add_subdirectory(3rdparty/msgpack)
add_subdirectory(kinematics)
add_subdirectory(utils)
add_subdirectory(option_framework)
add_subdirectory(sockethelper)

add_library(${PROJECT_NAME}
    ankle_balancer.cpp
    ankle_balancer.h
    arm_controller.cpp
    arm_controller.h
    odometry.h
    sit_motion.cpp
    sit_motion.h
    walking_engine.cpp
    walking_engine.h
    walking_time.h
    kalman_filter.cpp
    kalman_filter.h
)
target_link_libraries(${PROJECT_NAME} utils Kinematics msgpack-cxx OptionFramework)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
)
