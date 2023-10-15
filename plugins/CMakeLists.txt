cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(ControllerBuffer)

set(IGN_PLUGIN_VER 1)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

find_package(ignition-gazebo6 REQUIRED)
# Add sources for each plugin to be registered.
add_library(ControllerBuffer ControllerBuffer.cpp)
set_property(TARGET ControllerBuffer PROPERTY CXX_STANDARD 17)
target_link_libraries(ControllerBuffer
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)