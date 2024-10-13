project(SocketHelper)

find_package(Boost CONFIG COMPONENTS system REQUIRED QUIET)

add_library(${PROJECT_NAME}
    sockethelper.h
    sockethelper.cpp
)
target_link_libraries(${PROJECT_NAME} Boost::system)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
)
