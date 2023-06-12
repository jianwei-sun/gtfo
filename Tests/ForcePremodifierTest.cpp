#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(ForcePremodifierTest, FlipForces)
{
    gtfo::PointMassSecondOrder<2> system((gtfo::SecondOrderParameters<double>()));

    auto lambda = [](const Eigen::Vector2d& force, const gtfo::DynamicsBase<2>& system){
        return -force;
    };
    system.SetForcePremodifier(lambda);

    for(unsigned i = 0; i < 100; ++i){
        system.Step(Eigen::Vector2d::Ones());
    }
    EXPECT_TRUE((system.GetPosition().array() < 0.0).all());
}