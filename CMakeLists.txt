cmake_minimum_required(VERSION 3.10)
project(MoteusAPI VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLES "Build example executables" OFF)
option(BUILD_TESTS    "Build test executables"    OFF)
option(BUILD_SCRIPTS  "Build custom script executables" OFF)

# Helper to set per-target runtime dir
function(set_runtime_dir target outdir)
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${outdir}"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${outdir}"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${outdir}"
    )
endfunction()

# Add subdirs
add_subdirectory(src)

if(BUILD_SCRIPTS)
  add_subdirectory(scripts)
endif()

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
