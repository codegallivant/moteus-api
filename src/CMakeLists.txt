add_library(moteus_api STATIC
    moteus_api.cpp
)

target_compile_features(moteus_api PUBLIC cxx_std_20)

target_include_directories(moteus_api
    SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/moteus_drivers
)

target_include_directories(moteus_api
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/moteus_api
)