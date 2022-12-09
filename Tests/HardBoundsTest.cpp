#include <gtest/gtest.h>

#include "../gtfo.hpp"

TEST(HardBoundsTest, SecondOrderSystem2DInfNorm)
{
    gtfo::PointMassSecondOrder<2> system;
    system.SetParameters(gtfo::SecondOrderParameters<double>());
    system.SetHardBound(gtfo::NormBound<Eigen::Infinity, 2>(1.0));

    const Eigen::Vector2d force(1.0, 0.5);

    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force);
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}

TEST(HardBoundsTest, SecondOrderSystem2DRectangle)
{
    gtfo::PointMassSecondOrder<2> system;
    system.SetParameters(gtfo::SecondOrderParameters<double>());
    system.SetHardBound(gtfo::RectangleBound<2>(Eigen::Vector2d(-1.0, -1.0), Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(0.0, 0.0)));

    const Eigen::Vector2d force(1.0, 0.5);

    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force);
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}

TEST(HardBoundsTest, FirstOrderSystem2DInfNorm)
{
    gtfo::PointMassFirstOrder<2> system;
    system.SetParameters(gtfo::FirstOrderParameters<double>());
    system.SetHardBound(gtfo::NormBound<Eigen::Infinity, 2>(1.0));

    const Eigen::Vector2d force(2.0, 1.0);

    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force);
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}