#include <gtest/gtest.h>

// Tests that the environment has been set up correctly 
TEST(CMakeSmokeTest, BasicAssertions) {

  // Expect two strings not to be equal
  EXPECT_STRNE("hello", "world");
}