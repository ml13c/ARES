cmake_minimum_required(VERSION 3.10)

project(MissionPlanner)


set(CMAKE_CXX_STANDARD 17)


include_directories(include)


add_executable(
    mission_planner

    src/main.cpp
    src/Mission.cpp
)