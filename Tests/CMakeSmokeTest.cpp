#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <mujoco/mujoco.h>

// Tests that the environment has been set up correctly 
TEST(CMakeSmokeTest, BasicAssertions) {

  // Expect two strings not to be equal
  EXPECT_STRNE("hello", "world");
}

// Verifies version information for Eigen
TEST(CMakeSmokeTest, EigenVersion){
  EXPECT_TRUE(EIGEN_VERSION_AT_LEAST(3, 4, 0));
}

// Verifies that MuJoCo is compiled and linked correctly
TEST(CMakeSmokeTest, MuJoCoVersion){
  EXPECT_TRUE(mjVERSION_HEADER == mj_version());
  EXPECT_EQ(mjVERSION_HEADER, 231);
}