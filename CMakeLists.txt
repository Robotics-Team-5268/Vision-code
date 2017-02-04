cmake_minimum_required(VERSION 2.4)
project(Vision_code)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        GripPipeline.cpp
        RobotVision.cpp)

add_executable(Vision_code ${SOURCE_FILES})
target_link_libraries(Vision_code opencv_world)
target_link_libraries(Vision_code ntcore)