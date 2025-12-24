add_library(moteus_api STATIC
    moteus_api.cpp
)

target_include_directories(moteus_api
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
