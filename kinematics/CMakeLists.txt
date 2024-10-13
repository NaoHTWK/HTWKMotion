project(Kinematics)

add_library(${PROJECT_NAME}
    InverseKinematics.cpp
    InverseKinematics.h
    ForwardKinematic.cpp
    ForwardKinematic.h
    Joints.h
    LegJoints.h
    LegPosition.h
    Pose3D.cpp
    Pose3D.h
    PositionFeet.cpp
    PositionFeet.h
)
target_link_libraries(${PROJECT_NAME} utils Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
)
