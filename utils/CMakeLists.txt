project(utils CXX)

set(srcs
    exponential_moving_average.h
    fsr.h
    imu.h
    joints.h
    point_2d.h
    point_3d.h
    position.h
    stl_ext.h
    LegPositionPlain.h
    PositionPlain.h
    lola_frame.h
    lola_frame.cpp
    battery.h
    leds.h
    touch_sensors.h
)
add_library(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME} cpp17 msgpack-cxx)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17 LINKER_LANGUAGE CXX)
