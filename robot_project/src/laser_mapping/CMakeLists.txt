cmake_minimum_required(VERSION 2.8.3)
project(laser_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH /usr/local/lib/cmake/)

catkin_package(
    )

add_executable(key_motion
    src/key_motion_node.cpp)

target_link_libraries(key_motion ${catkin_LIBRARIES})
