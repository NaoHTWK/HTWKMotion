project(cpp17 CXX)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE .)

if (CMAKE_CROSSCOMPILING)
    target_include_directories(${PROJECT_NAME} INTERFACE ./target)
    target_link_libraries(${PROJECT_NAME} INTERFACE absl::base)
endif()
