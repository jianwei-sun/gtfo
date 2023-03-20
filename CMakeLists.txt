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

#----------------------------------------------------------------------------------------------------
# Third party libraries
#----------------------------------------------------------------------------------------------------
# Eigen 3.4.0
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# MuJoCo 2.3.1
find_package(mujoco 2.3.1 REQUIRED)

#----------------------------------------------------------------------------------------------------
# Project subdirectories
#----------------------------------------------------------------------------------------------------
enable_testing()
add_subdirectory(Tests)