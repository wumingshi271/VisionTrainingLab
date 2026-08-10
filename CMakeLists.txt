cmake_minimum_required(VERSION 4.2)
project(VisionTrainingLab)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
include_directories(include/task01)

add_executable(VisionTrainingLab
        include/task01/armor_pose_solver.hpp
        src/task01/armor_pose_solver.cpp
        src/task01/task01_main.cpp)
