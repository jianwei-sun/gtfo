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

# Default is debug mode if no build type is set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Determine the type of configurator of the generator
get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(isMultiConfig)
    set(CONFIG_DEPENDENT_OUTPUT_DIR "${CMAKE_BUILD_TYPE}")
else()
    set(CONFIG_DEPENDENT_OUTPUT_DIR "")
endif()

# Build options
option(GTFO_BUILD_TESTS "Build tests for GTFO" ON)

#----------------------------------------------------------------------------------------------------
# Third party libraries
#----------------------------------------------------------------------------------------------------
# Eigen 3.4.0
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# MuJoCo 2.3.1
find_package(mujoco 2.3.1)

# osqp 0.6.2
find_package(osqp REQUIRED)

#----------------------------------------------------------------------------------------------------
# gtfo library
#----------------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} INTERFACE
    ${CMAKE_DL_LIBS}
    Eigen3::Eigen
    osqp::osqpstatic
)

if(mujoco_FOUND)
    target_link_libraries(${PROJECT_NAME} INTERFACE
        mujoco::mujoco
    )
endif()

#----------------------------------------------------------------------------------------------------
# Testing
#----------------------------------------------------------------------------------------------------
if(GTFO_BUILD_TESTS)
    enable_testing()
    add_subdirectory(Tests)
endif()