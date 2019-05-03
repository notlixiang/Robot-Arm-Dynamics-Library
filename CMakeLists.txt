cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project(robot_dynamics_lib)

set(EIGEN_INC_DIR /usr/include/eigen3/)
include_directories(${EIGEN_INC_DIR})

include_directories(
        ${PROJECT_SOURCE_DIR}
)
add_library(RobotDynamics SHARED RobotDynamics.cpp)
