#----------------------------------------------------------------------------------------------------
# File: CMakeLists.txt
# Date: 12/03/2022
# Desc: Top level CMake configuration file for the gtfo project
# Usage: Configure this file appropriately. Then in the top level directory:
#           `mkdir build`
#           `cd build`
#           `cmake ..` 
#           `cmake --build . -j4`
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

# Sets the install prefix dirctory. Otherwise, pass in to cmake through the -DCMAKE_INSTALL_PREFIX flag on the command line
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH ${CMAKE_SOURCE_DIR} FORCE)
endif()

# Sets an absolute install path
set(INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)

#----------------------------------------------------------------------------------------------------
# Third party libraries
#----------------------------------------------------------------------------------------------------
# Eigen 3.4.0
include_directories(${PROJECT_SOURCE_DIR}/External/eigen)

# MuJoCo 2.3.1
set(mujoco_DIR ${PROJECT_SOURCE_DIR}/External/mujoco/bin)
find_package(mujoco PATHS ${mujoco_DIR} REQUIRED)
include_directories(${mujoco_INCLUDE_DIR})

#----------------------------------------------------------------------------------------------------
# Project subdirectories
#----------------------------------------------------------------------------------------------------
enable_testing()
add_subdirectory(Tests)