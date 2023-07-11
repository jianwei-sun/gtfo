#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(CollisionAvoidanceTest, Unconstrained)
{
    gtfo::collision::Manipulator<4, 1> manipulator(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 2.0, 3.0)
    });
    gtfo::collision::Obstacle obstacle(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d::Zero()
    });
    
    manipulator.ComputeCollisions(obstacle, 0.1);

    const Eigen::Vector4d desired_joint_space_velocity = Eigen::Vector4d::Ones();
    const Eigen::Vector4d actual_joint_space_velocity = manipulator.GetSafeJointSpaceVelocity(desired_joint_space_velocity);

    EXPECT_TRUE(gtfo::IsEqual(desired_joint_space_velocity, actual_joint_space_velocity));
}

TEST(CollisionAvoidanceTest, ZeroVelocity)
{
    gtfo::collision::Manipulator<4, 1> manipulator(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 2.0, 3.0)
    });
    gtfo::collision::Obstacle obstacle(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d::Zero()
    });

    manipulator.ComputeCollisions(obstacle, 0.1);

    manipulator.SetJointFixed(0, true);
    manipulator.SetJointFixed(1, false);
    manipulator.SetJointFixed(2, true);
    manipulator.SetJointFixed(3, false);

    const Eigen::Vector4d actual_joint_space_velocity = manipulator.GetSafeJointSpaceVelocity(Eigen::Vector4d::Ones());

    EXPECT_TRUE(gtfo::IsEqual(Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), actual_joint_space_velocity));
}