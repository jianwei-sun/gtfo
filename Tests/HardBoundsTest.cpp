#include <gtest/gtest.h>

#include "../gtfo.hpp"

TEST(HardBoundsTest, SecondOrderSystem2D) {
    gtfo::PointMassSecondOrder<2> system;
    system.SetParameters(gtfo::SecondOrderParameters<double>(1.0, 1.0, 1.0));
    system.SetHardBound(gtfo::NormBound<Eigen::Infinity, 2>(1.0));

    const Eigen::Vector2d force(1.0, 0.5);

    for(size_t i = 0; i < 10; ++i){
        std::cout << "Pos: " << system.GetPosition().transpose() << ", Vel: " << system.GetVelocity().transpose() << "\n";
        system.Step(force);
    }

    std::cout << "Pos: " << system.GetPosition().transpose() << ", Vel: " << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d(1.0, 1.0)));

    std::cout << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));

    std::cout << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}