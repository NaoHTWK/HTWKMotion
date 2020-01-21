project(WalkingEngine CXX)

add_subdirectory(cpp17)
add_subdirectory(eigen)
add_subdirectory(kinematics)
add_subdirectory(utils)

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
)
target_link_libraries(${PROJECT_NAME} utils Kinematics)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
)
