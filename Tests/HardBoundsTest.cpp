#include <gtest/gtest.h>

#include "../gtfo.hpp"

// Verifies that a point mass with second order dynamics is correctly driven to the corner of the rectangle bound
TEST(HardBoundsTest, SecondOrderSystem2DRectangle)
{
    gtfo::PointMassSecondOrder<2> system;
    system.SetParameters(gtfo::SecondOrderParameters<double>());
    system.SetHardBound(gtfo::RectangleBound<2>(Eigen::Vector2d::Ones()));

    const Eigen::Vector2d force(1.0, 0.5);

    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force);
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}

// Similar verification, but for a point mass with first order dynamics
TEST(HardBoundsTest, FirstOrderSystem2DRectangle) {
    gtfo::PointMassFirstOrder<2> system;
    system.SetParameters(gtfo::FirstOrderParameters<double>());
    system.SetHardBound(gtfo::RectangleBound<2>(Eigen::Vector2d::Ones()));

    // Because the first order system acts as a low pass filter, the steady state position will be equal to the force input
    // So, the force input is set as a value past the rectangular bound, in order to test the hard bound
    const Eigen::Vector2d force(2.0, 1.0);

    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force);
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}