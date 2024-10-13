project(OptionFramework)

add_library(${PROJECT_NAME}
    booloption.h
    booloption.cpp
    enumoption.h
    enumoption.cpp
    floatoption.h
    floatoption.cpp
    intoption.cpp
    intoption.h
    option.h
    option.cpp
    optionset.h
    optionset.cpp
    robotoption.h
    robotoption.cpp
)

target_link_libraries(${PROJECT_NAME} SocketHelper utils)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
