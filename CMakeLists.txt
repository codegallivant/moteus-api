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
    moteus_api/moteus_api.cpp
)
target_link_libraries(read PRIVATE Threads::Threads)

add_executable(write
    tests/write.cpp
    moteus_api/moteus_api.cpp
)

# add_executable(load
#     scripts/load.cpp
#     moteus_api/moteus_api.cpp
# )

