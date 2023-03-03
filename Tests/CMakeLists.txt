#----------------------------------------------------------------------------------------------------
# File: Tests/CMakeLists.txt
# Date: 12/03/2022
# Desc: CMakeList for test files
# Usage: Each time a new test file is added, simply run `cmake --build . -j4` in the build directory again.
#        Tests can be executed by running `ctest` in the build directory
#        Specific tests can be executed by running `ctest -R YourTestName`
#----------------------------------------------------------------------------------------------------
project(gtfo-tests)

include(FetchContent)

# Googletest 1.12.1
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.12.1
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Include the testing framework
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# Find all test files
file(GLOB_RECURSE TEST_FILES
    ${PROJECT_SOURCE_DIR}/*.cpp
)

# Create the testing executables
add_executable(
    ${PROJECT_NAME}
    ${TEST_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    GTest::gtest_main
    mujoco::mujoco
    ${CMAKE_SOURCE_DIR}/External/glfw-3.3.8.bin.MACOS/lib-x86_64/libglfw3.a
)

# Copy the MuJoCo runtime, which is only on Windows
if(WIN32)
    file(INSTALL ${mujoco_DIR}/../../../bin/mujoco.dll
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()

# Copy the MuJoCo models to the test directory
file(GLOB MUJOCO_MODELS ${CMAKE_SOURCE_DIR}/Wrapper/Models/*.xml)
file(INSTALL ${MUJOCO_MODELS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Automatically find all tests under the Google Test framework
gtest_discover_tests(${PROJECT_NAME})
