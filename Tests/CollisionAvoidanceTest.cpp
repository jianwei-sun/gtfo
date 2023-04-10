#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(CollisionAvoidanceTest, Unconstrained)
{
    gtfo::collision::Manipulator<4> manipulator(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 2.0, 3.0)
    });
    gtfo::collision::Obstacle obstacle(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d::Zero()
    });
    
    manipulator.ComputeCollisions(obstacle, 0.1);

    const Eigen::Vector4d desired_joint_space_velocity = Eigen::Vector4d::Ones();
    const Eigen::Vector4d actual_joint_space_Velocity = manipulator.GetSafeJointSpaceVelocity(desired_joint_space_velocity);

    EXPECT_TRUE(gtfo::IsEqual(desired_joint_space_velocity, actual_joint_space_Velocity));
}

TEST(CollisionAvoidanceTest, VelocityLimited)
{
    gtfo::collision::Manipulator<4> manipulator(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 2.0, 3.0)
    });

    manipulator.EnableJointVelocityLimits(Eigen::Vector4d::Constant(-0.1), Eigen::Vector4d::Constant(0.1));

    const Eigen::Vector4d desired_joint_space_velocity = Eigen::Vector4d::Ones();
    const Eigen::Vector4d actual_joint_space_Velocity = manipulator.GetSafeJointSpaceVelocity(desired_joint_space_velocity);

    EXPECT_TRUE(gtfo::IsEqual(Eigen::Vector4d::Constant(0.1), actual_joint_space_Velocity));
}

TEST(CollisionAvoidanceTest, VelocityZeroed)
{
    gtfo::collision::Manipulator<4> manipulator(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 2.0, 3.0)
    });

    manipulator.EnableJointVelocityLimits(Eigen::Vector4d::Ones(), Eigen::Vector4d::Ones());

    const Eigen::Vector4d desired_joint_space_velocity = -Eigen::Vector4d::Ones();
    const Eigen::Vector4d actual_joint_space_Velocity = manipulator.GetSafeJointSpaceVelocity(desired_joint_space_velocity);

    EXPECT_TRUE(gtfo::IsEqual(Eigen::Vector4d::Ones(), actual_joint_space_Velocity));
}