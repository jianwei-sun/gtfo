#include <gtest/gtest.h>
#include "gtfo.hpp"

// Verifies that dynamics can be paused and that position and velocity are correct
TEST(DynamicsPauseTest, SecondOrderSystem)
{
    gtfo::PointMassSecondOrder<3> system((gtfo::SecondOrderParameters<double>()));

    const Eigen::Vector3d force(1.0, 0.0, -1.0);

    // Step with the force while the system's dynamics are paused
    system.PauseDynamics(true);
    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force);
    }

    EXPECT_TRUE(system.DynamicsArePaused());
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector3d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector3d::Zero()));

    // Pass in a physical position while calling paused
    const Eigen::Vector3d physical_position(1.0, 2.0, 3.0);
    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force, physical_position);
    }

    EXPECT_TRUE(system.DynamicsArePaused());
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), physical_position));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector3d::Zero()));

    // Now unpause the system
    system.PauseDynamics(false);
    system.Step(force);

    EXPECT_FALSE(system.DynamicsArePaused());
    EXPECT_FALSE(gtfo::IsEqual(system.GetPosition(), physical_position));
    EXPECT_FALSE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
    EXPECT_FALSE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector3d::Zero()));
}

// Repeat the test for a first order system
TEST(DynamicsPauseTest, FirstOrderSystem)
{
    gtfo::PointMassFirstOrder<3> system((gtfo::FirstOrderParameters<double>()));

    const Eigen::Vector3d force(1.0, 0.0, -1.0);

    // Step with the force while the system's dynamics are paused
    system.PauseDynamics(true);
    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force);
    }

    EXPECT_TRUE(system.DynamicsArePaused());
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector3d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector3d::Zero()));

    // Pass in a physical position while calling paused
    const Eigen::Vector3d physical_position(1.0, 2.0, 3.0);
    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force, physical_position);
    }

    EXPECT_TRUE(system.DynamicsArePaused());
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), physical_position));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector3d::Zero()));

    // Now unpause the system
    system.PauseDynamics(false);
    system.Step(force);

    EXPECT_FALSE(system.DynamicsArePaused());
    EXPECT_FALSE(gtfo::IsEqual(system.GetPosition(), physical_position));
    EXPECT_FALSE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
    EXPECT_FALSE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector3d::Zero()));
}