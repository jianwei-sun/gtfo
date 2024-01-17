#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(RigidBodyDynamicsTest, DynamicsVector)
{  
    using Vector3 = Eigen::Vector3f;
    using Vector6 = Eigen::Matrix<float, 6, 1>;
    using Vector18 = Eigen::Matrix<float, 18, 1>;

    // Create three standalong systems
    gtfo::RigidBodySecondOrder<float> system1(0.1f, 3.0f, Vector3(0.001f, 0.002f, 0.003f), 0.4f, 0.6f);
    gtfo::RigidBodySecondOrder<float> system2(0.05f, 1.0f, Vector3(0.003f, 0.001f, 0.002f), 0.2f, 0.0f);
    gtfo::PointMassSecondOrder<6, float> system3(gtfo::SecondOrderParameters<float>(0.12f, 1.2f, 0.3f));

    // Copy them into a vector
    gtfo::DynamicsVector<
        gtfo::RigidBodySecondOrder<float>,
        gtfo::RigidBodySecondOrder<float>,
        gtfo::PointMassSecondOrder<6, float>
    > vector_system(system1, system2, system3);

    // Propagate both and compare the states
    const Vector6 input1 = (Vector6() << -0.1f, 0.0f, 0.1f, -0.1f, 0.1f, 3.2f).finished();
    const Vector6 input2 = (Vector6() << 3.0f, 5.0f, -2.0f, 1.5f, -2.5f, 3.5f).finished();
    const Vector6 input3 = (Vector6() << 0.0f, 0.0f, 0.3f, -0.3f, 0.0f, 0.25f).finished();

    for(const Vector18& total_input : {
        (Vector18() << input1, input2, input3).finished(),
        (Vector18() << input2, input3, input1).finished(),
        (Vector18() << input3, input1, input2).finished()
    }){
        system1.Step(total_input.head<6>());
        system2.Step(total_input.segment<6>(6));
        system3.Step(total_input.tail<6>());
        vector_system.Step(total_input);

        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetPosition().head<7>(), system1.GetPosition()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetOldPosition().head<7>(), system1.GetOldPosition()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetVelocity().head<6>(), system1.GetVelocity()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetAcceleration().head<6>(), system1.GetAcceleration()));

        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetPosition().segment<7>(7), system2.GetPosition()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetOldPosition().segment<7>(7), system2.GetOldPosition()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetVelocity().segment<6>(6), system2.GetVelocity()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetAcceleration().segment<6>(6), system2.GetAcceleration()));

        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetPosition().tail<6>(), system3.GetPosition()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetOldPosition().tail<6>(), system3.GetOldPosition()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetVelocity().tail<6>(), system3.GetVelocity()));
        EXPECT_TRUE(gtfo::IsEqual(vector_system.GetAcceleration().tail<6>(), system3.GetAcceleration()));
    }
}

TEST(RigidBodyDynamicsTest, DynamicsSelector)
{
    using Vector3 = Eigen::Vector3f;
    using Vector6 = Eigen::Matrix<float, 6, 1>;

    // Create two identical systems as well as a selector of the two
    gtfo::RigidBodySecondOrder<float> system1(0.1f, 3.0f, Vector3(0.001f, 0.002f, 0.003f), 0.4f, 0.6f);
    gtfo::RigidBodySecondOrder<float> system2(system1);

    gtfo::DynamicsSelector<
        gtfo::RigidBodySecondOrder<float>,
        gtfo::RigidBodySecondOrder<float>
    > selector_system(system1, system2);

    // Step the first one and ensure that the second one's states are updated when it's selected
    const Vector6 input1 = (Vector6() << -0.1f, 0.0f, 0.1f, -0.1f, 0.1f, 3.2f).finished();
    system1.Step(input1);
    system2.Step(input1);
    selector_system.Step(input1);
    selector_system.Select(1);

    EXPECT_TRUE(gtfo::IsEqual(selector_system.GetPosition(), system2.GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(selector_system.GetOldPosition(), system2.GetOldPosition()));
    EXPECT_TRUE(gtfo::IsEqual(selector_system.GetVelocity(), system2.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(selector_system.GetAcceleration(), system2.GetAcceleration()));

    // Now repeat in the other direction
    const Vector6 input2 = (Vector6() << 3.0f, 5.0f, -2.0f, 1.5f, -2.5f, 3.5f).finished();
    system1.Step(input2);
    system2.Step(input2);
    selector_system.Step(input2);
    selector_system.Select(0);

    EXPECT_TRUE(gtfo::IsEqual(selector_system.GetPosition(), system1.GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(selector_system.GetOldPosition(), system1.GetOldPosition()));
    EXPECT_TRUE(gtfo::IsEqual(selector_system.GetVelocity(), system1.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(selector_system.GetAcceleration(), system1.GetAcceleration()));
}