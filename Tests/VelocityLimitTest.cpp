#include <gtest/gtest.h>

#include "../gtfo.hpp"

// Verifies a velocity limit of zero works correctly
TEST(VelocityLimitTest, ZeroLimit)
{
    gtfo::PointMassSecondOrder<2> system((gtfo::SecondOrderParameters<double>()));
    system.SetVelocityLimit(0.0);

    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(Eigen::Vector2d(1.0, -1.0));
    }

    std::cout << "Final velocity: " << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));

    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}

// Verifies that an unlimited velocity works correctly
TEST(VelocityLimitTest, NoLimit)
{
    using Scalar = Eigen::Matrix<double, 1, 1>;

    gtfo::PointMassSecondOrder<1> system((gtfo::SecondOrderParameters<double>()));

    for (size_t i = 0; i < 3; ++i)
    {
        system.Step(Scalar(1.0));
    }

    const double inv_exp = std::exp(-1.0);

    std::cout << "Final velocity: " << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Scalar((1 - inv_exp) * (inv_exp * (1 + inv_exp) + 1))));

    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Scalar(-(1 - inv_exp) * (inv_exp * (1 + inv_exp) + 1) + 1)));
}

// Verifies that a limited velocity works correctly
TEST(VelocityLimitTest, ArbitraryLimit)
{
    using Scalar = Eigen::Matrix<double, 1, 1>;

    gtfo::PointMassSecondOrder<1> system((gtfo::SecondOrderParameters<double>()));
    system.SetVelocityLimit(0.1);

    for (size_t i = 0; i < 3; ++i)
    {
        system.Step(Scalar(1.0));
    }

    std::cout << "Final velocity: " << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Scalar(0.1)));

    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Scalar(0.0)));
}