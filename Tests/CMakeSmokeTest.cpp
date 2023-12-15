#include <gtest/gtest.h>
#include <Eigen/Dense>

// Tests that the environment has been set up correctly 
TEST(CMakeSmokeTest, BasicAssertions) {

  // Expect two strings not to be equal
  EXPECT_STRNE("hello", "world");
}

// Verifies version information for Eigen
TEST(CMakeSmokeTest, EigenVersion){
  EXPECT_TRUE(EIGEN_VERSION_AT_LEAST(3, 4, 0));
}
