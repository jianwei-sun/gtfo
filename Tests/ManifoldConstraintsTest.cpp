#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(ManifoldConstraintsTest, PassThroughTest)
{
    gtfo::ManifoldConstraints<4, 2> manifold_constraints;

    for(unsigned i = 0; i < 10; ++i){
        Eigen::Vector4d force = Eigen::Vector4d::Ones() * i;
        EXPECT_TRUE(gtfo::IsEqual(manifold_constraints.Step(force, Eigen::Vector4d::Ones(), Eigen::Vector4d::Ones()) , force));
    }

}