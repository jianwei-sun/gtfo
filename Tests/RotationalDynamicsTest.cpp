#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(RotationalDynamicsTest, QuaternionStorage)
{
    const Eigen::Vector4d vector = Eigen::Vector4d(1.0, 2.0, 3.0, 4.0).normalized();
    const Eigen::Quaterniond quaternion = Eigen::Quaterniond(vector);
    const gtfo::RotationSecondOrder<double> system(0.1, Eigen::Vector3d::Ones(), 1.0, quaternion);

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), vector));
    EXPECT_NEAR(system.GetOrientation().norm(), 1.0, 1e-15);
    EXPECT_NEAR(system.GetOrientation().angularDistance(quaternion), 0.0, 1e-15);
}

TEST(RotationalDynamicsTest, RotateAround)
{
    // System starts at the identity
    gtfo::RotationSecondOrder<double> system(1.0, Eigen::Vector3d::Ones(), 0.0);
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector4d::UnitW()));

    // Apply torque 90 degrees about X
    system.Step(Eigen::Vector3d::UnitX() * M_PI_2);
    EXPECT_NEAR(system.GetOrientation().norm(), 1.0, 1e-15);
    EXPECT_TRUE(gtfo::IsEqual(system.GetOrientation().coeffs(), Eigen::Vector4d(std::sqrt(2) / 2, 0, 0, std::sqrt(2) / 2)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d(M_PI_2, 0, 0)));

    // Apply the opposite torque. Since implicit Euler is used, the position shouldn't advance
    system.Step(-Eigen::Vector3d::UnitX() * M_PI_2);
    EXPECT_NEAR(system.GetOrientation().norm(), 1.0, 1e-15);
    EXPECT_TRUE(gtfo::IsEqual(system.GetOrientation().coeffs(), Eigen::Vector4d(std::sqrt(2) / 2, 0, 0, std::sqrt(2) / 2)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));

    // Now rotate back to the original orientation
    system.Step(Eigen::Vector3d::UnitX() * M_PI_2);
    system.Step(Eigen::Vector3d::Zero());
    system.Step(Eigen::Vector3d::Zero());
    system.Step(-Eigen::Vector3d::UnitX() * M_PI_2);
    EXPECT_NEAR(system.GetOrientation().norm(), 1.0, 1e-15);
    EXPECT_NEAR(system.GetOrientation().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-15);
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
}

TEST(RotationalDynamicsTest, RightAngleRotations)
{
    gtfo::RotationSecondOrder<double> system(1.0, Eigen::Vector3d::Ones(), 0.0);

    // Rotate 90 degrees about X and stop
    system.Step(Eigen::Vector3d::UnitX() * M_PI_2);
    system.Step(-Eigen::Vector3d::UnitX() * M_PI_2);
    EXPECT_TRUE(gtfo::IsEqual(system.GetOrientation().coeffs(), Eigen::Vector4d(std::sqrt(2) / 2, 0, 0, std::sqrt(2) / 2)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));

    // Now rotate 90 degrees about the body Y and stop
    system.Step(Eigen::Vector3d::UnitY() * M_PI_2);
    system.Step(-Eigen::Vector3d::UnitY() * M_PI_2);
    EXPECT_TRUE(gtfo::IsEqual(system.GetOrientation().coeffs(), Eigen::Vector4d(0.5, 0.5, 0.5, 0.5)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
}

TEST(RotationalDynamicsTest, Damping)
{
    gtfo::RotationSecondOrder<double> system(0.01, Eigen::Vector3d::Ones(), 1.0);

    // Accelerate the system
    for(unsigned i = 0; i < 200; ++i){
        system.Step(Eigen::Vector3d(1.0, 1.0, 0.0));
    }
    EXPECT_FALSE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
    EXPECT_NEAR(system.GetOrientation().norm(), 1.0, 1e-15);

    // Let the system slow down by itself
    for(unsigned i = 0; i < 1000; ++i){
        system.Step(Eigen::Vector3d::Zero());
    }
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector3d::Zero()));
    EXPECT_NEAR(system.GetOrientation().norm(), 1.0, 1e-15);
}

TEST(RotationalDynamicsTest, SmallAngleApproximation)
{
    gtfo::RotationSecondOrder<double> system(0.01, Eigen::Vector3d::Ones(), 0.0);
    for(unsigned i = 0; i < 10000; ++i){
        system.Step(Eigen::Vector3d(0.001, 0, 0));
    }
    EXPECT_NEAR(system.GetOrientation().norm(), 1.0, 1e-15);
    EXPECT_NEAR(system.GetOrientation().y(), 0.0, 1e-15);
    EXPECT_NEAR(system.GetOrientation().z(), 0.0, 1e-15);
}