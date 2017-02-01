cmake_minimum_required(VERSION 3.6)
project(tests)

set(CMAKE_CXX_STANDARD 11 -lopencv_world)

set(SOURCE_FILES
        GripPipeline.cpp
        GripPipeline.h
        RobotVision.cpp
        RobotVision.hpp)

add_executable(tests ${SOURCE_FILES})