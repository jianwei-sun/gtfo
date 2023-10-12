#include <gtest/gtest.h>
#include "gtfo.hpp"
#include <osqp.h>

TEST(CollisionAvoidanceTest, Unconstrained)
{
    using Vector3 = Eigen::Matrix<c_float, 3, 1>;
    using Vector4 = Eigen::Matrix<c_float, 4, 1>;

    gtfo::collision::Manipulator<4, 1, 4, c_float> manipulator(std::vector<Vector3>{
        Vector3(1.0, 2.0, 3.0)
    });
    gtfo::collision::Obstacle<c_float> obstacle(std::vector<Vector3>{
        Vector3::Zero()
    });
    
    manipulator.ComputeCollisions(obstacle, 0.1);

    const Vector4 desired_joint_space_velocity = Vector4::Ones();
    const Vector4 actual_joint_space_velocity = manipulator.GetSafeJointSpaceVelocity(desired_joint_space_velocity);

    EXPECT_TRUE(gtfo::IsEqual(desired_joint_space_velocity, actual_joint_space_velocity));
}

TEST(CollisionAvoidanceTest, ZeroVelocity)
{
    using Vector3 = Eigen::Matrix<c_float, 3, 1>;
    using Vector4 = Eigen::Matrix<c_float, 4, 1>;

    gtfo::collision::Manipulator<4, 1, 4, c_float> manipulator(std::vector<Vector3>{
        Vector3(1.0, 2.0, 3.0)
    });
    gtfo::collision::Obstacle<c_float> obstacle(std::vector<Vector3>{
        Vector3::Zero()
    });

    manipulator.ComputeCollisions(obstacle, 0.1);

    manipulator.SetJointFixed(0, true);
    manipulator.SetJointFixed(1, false);
    manipulator.SetJointFixed(2, true);
    manipulator.SetJointFixed(3, false);

    const Vector4 actual_joint_space_velocity = manipulator.GetSafeJointSpaceVelocity(Vector4::Ones());

    EXPECT_TRUE(gtfo::IsEqual(Vector4(0.0, 1.0, 0.0, 1.0), actual_joint_space_velocity));
}