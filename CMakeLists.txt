#----------------------------------------------------------------------------------------------------
# File: CMakeLists.txt
# Desc: Top level CMake configuration file for the gtfo project
# Usage: Configure this file appropriately. Then in the top level directory:
#           `mkdir build`
#           `cd build`
#           `cmake ..` 
#           `cmake --build .`
#       Tests can be invoked by calling either:
#           `ctest` to run all the tests
#           `ctest -R YourTestName` to run a specific test
#----------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.14)
project(gtfo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

# Build options
option(GTFO_BUILD_TESTS "Build tests for GTFO" ON)

#----------------------------------------------------------------------------------------------------
# Third party libraries
#----------------------------------------------------------------------------------------------------
# Eigen 3.4.0
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# MuJoCo 2.3.1
find_package(mujoco 2.3.1 REQUIRED)

# osqp 0.6.2
if(WIN32)
    find_package(osqp REQUIRED)
else()
    add_subdirectory(${PROJECT_SOURCE_DIR}/External/osqp)
endif()

# abseil-cpp
add_subdirectory(${PROJECT_SOURCE_DIR}/External/abseil-cpp)

#----------------------------------------------------------------------------------------------------
# gtfo library
#----------------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME}
    PRIVATE
        ${PROJECT_SOURCE_DIR}/External/osqp-cpp/src/osqp++.cc
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/External/osqp-cpp/include
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        Eigen3::Eigen
        mujoco::mujoco
        absl::base
        absl::synchronization
        absl::strings
        absl::status
        absl::statusor
        absl::span
        osqp::osqpstatic
)

#----------------------------------------------------------------------------------------------------
# Testing
#----------------------------------------------------------------------------------------------------
if(GTFO_BUILD_TESTS)
    enable_testing()
    add_subdirectory(Tests)
endif()