#include <gtest/gtest.h>
#include "gtfo.hpp"

// Verifies a velocity limit of zero works correctly
TEST(AccelerationLimitTest, ZeroLimit)
{
    gtfo::PointMassSecondOrder<2> system((gtfo::SecondOrderParameters<double>()));
    system.SetAccLimit(0.0);

    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(Eigen::Vector2d(1.0, -1.0));
    }


    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}

// Verifies that an unlimited acceleration works correctly
TEST(AccelerationLimitTest, NoLimit)
{
    using Scalar = Eigen::Matrix<double, 1, 1>;

    gtfo::PointMassSecondOrder<1> system((gtfo::SecondOrderParameters<double>()));

    for (size_t i = 0; i < 3; ++i)
    {
        system.Step(Scalar(1.0));
    }

    const double inv_exp = std::exp(-1.0);

    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Scalar(-(1 - inv_exp) * (inv_exp * (1 + inv_exp) + 1) + 1)));
}

// Verifies that a limited velocity works correctly
TEST(AccelerationLimitTest, ArbitraryLimit)
{
    using Scalar = Eigen::Matrix<double, 1, 1>;

    gtfo::PointMassSecondOrder<1> system((gtfo::SecondOrderParameters<double>()));
    system.SetAccLimit(0.1);

    for (size_t i = 0; i < 3; ++i)
    {
        system.Step(Scalar(100.0));
    }


    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Scalar(0.1)));
}