#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(CollisionAvoidanceTest, Unconstrained)
{
    gtfo::collision::Manipulator<4, 1, 4, float> manipulator(std::vector<Eigen::Vector3f>{
        Eigen::Vector3f(1.0, 2.0, 3.0)});
    gtfo::collision::Obstacle<float> obstacle(std::vector<Eigen::Vector3f>{
        Eigen::Vector3f::Zero()});

    manipulator.SetJointFixed(0, 1);
    manipulator.SetJointFixed(1, 1);
    manipulator.SetJointFixed(2, 1);
    manipulator.SetJointFixed(3, 1);

    manipulator.ComputeCollisions(obstacle, 0.1);

    const Eigen::Vector4f desired_joint_space_velocity = Eigen::Vector4f::Ones();
    const Eigen::Vector4f actual_joint_space_Velocity = manipulator.GetSafeJointSpaceVelocity(desired_joint_space_velocity);

    EXPECT_TRUE(gtfo::IsEqual(Eigen::Vector4f::Zero(), actual_joint_space_Velocity));
}