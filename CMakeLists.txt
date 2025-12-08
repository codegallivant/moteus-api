cmake_minimum_required(VERSION 3.10)

project(MoteusHardwareInterface VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_SOURCE_DIR}/includes
)

find_package(Threads REQUIRED)

add_executable(read
    tests/read.cpp
)
target_link_libraries(read PRIVATE Threads::Threads)

add_executable(constant_acceleration_trajectory
    tests/constant_acceleration_trajectory.cpp
)
target_link_libraries(constant_acceleration_trajectory PRIVATE Threads::Threads)

add_executable(velocity_control
    tests/velocity_control.cpp
)
target_link_libraries(velocity_control PRIVATE Threads::Threads)

add_executable(torque_control
    tests/torque_control.cpp
)
target_link_libraries(torque_control PRIVATE Threads::Threads)