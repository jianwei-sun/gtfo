#include <gtest/gtest.h>

#include "../gtfo.hpp"


TEST(DynamicsVectorTest, SingleModel)
{
    gtfo::SecondOrderParameters<double> parameters;
    gtfo::PointMassSecondOrder<2> system(parameters);
    gtfo::DynamicsVector<gtfo::PointMassSecondOrder<2>> system_vector(system);

    const Eigen::Vector2d force(1.0, -0.5);

    for(unsigned i = 0; i < 5; ++i){
        system.Step(force);
        system_vector.Step(force);
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), system_vector.GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), system_vector.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), system_vector.GetAcceleration()));

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), system_vector.GetModel<0>().GetPosition()));
}

TEST(DynamicsVectorTest, MultipleModels)
{
    using VectorN = Eigen::Matrix<double, 6, 1>;

    gtfo::FirstOrderParameters<double> parameters_1st;
    gtfo::SecondOrderParameters<double> parameters_2nd;
    gtfo::PointMassSecondOrder<1> system_1d(parameters_2nd);
    gtfo::PointMassFirstOrder<2> system_2d(parameters_1st);
    gtfo::PointMassSecondOrder<3> system_3d(parameters_2nd);

    // Construct a DynamicsVector by specifying the dimension of each model that goes in
    gtfo::DynamicsVector<
        gtfo::PointMassSecondOrder<1>,
        gtfo::PointMassFirstOrder<2>,
        gtfo::PointMassSecondOrder<3>> 
    system_vector(system_1d, system_2d, system_3d);

    const VectorN force = (VectorN() << 1.0, -0.5, 2.0, -2.0, -1.0, 0.0).finished();

    // Since the models are copied, step both DynamicsVector and the original models
    for(unsigned i = 0; i < 5; ++i){
        system_1d.Step(force.head<1>());
        system_2d.Step(force.block<2,1>(1,0));
        system_3d.Step(force.tail<3>());
        system_vector.Step(force);
    }

    // Verify that the states are the same
    const VectorN combined_position = (VectorN() << system_1d.GetPosition(), system_2d.GetPosition(), system_3d.GetPosition()).finished();
    const VectorN combined_velocity = (VectorN() << system_1d.GetVelocity(), system_2d.GetVelocity(), system_3d.GetVelocity()).finished();
    const VectorN combined_acceleration = (VectorN() << system_1d.GetAcceleration(), system_2d.GetAcceleration(), system_3d.GetAcceleration()).finished();

    EXPECT_TRUE(gtfo::IsEqual(combined_position, system_vector.GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(combined_velocity, system_vector.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(combined_acceleration, system_vector.GetAcceleration()));

    EXPECT_TRUE(gtfo::IsEqual(system_1d.GetPosition(), system_vector.GetModel<0>().GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(system_2d.GetPosition(), system_vector.GetModel<1>().GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(system_3d.GetPosition(), system_vector.GetModel<2>().GetPosition()));
}

TEST(DynamicsVectorTest, NestedVectors)
{
    using VectorN = Eigen::Matrix<double, 6, 1>;

    // Construct a DynamicsVector with a 1d and 2d model
    gtfo::FirstOrderParameters<double> parameters_1st;
    gtfo::SecondOrderParameters<double> parameters_2nd;
    gtfo::PointMassSecondOrder<1> system_1d(parameters_2nd);
    gtfo::PointMassFirstOrder<2> system_2d(parameters_1st);
    gtfo::DynamicsVector<
        gtfo::PointMassSecondOrder<1>,
        gtfo::PointMassFirstOrder<2>> 
    vector_system_3d(system_1d, system_2d);

    // Nest a 3d DynamicsVector in a 6d DynamicsVector
    gtfo::PointMassSecondOrder<3> system_3d(parameters_2nd);
    gtfo::DynamicsVector<
        gtfo::DynamicsVector<
            gtfo::PointMassSecondOrder<1>,
            gtfo::PointMassFirstOrder<2>>,
        gtfo::PointMassSecondOrder<3>> 
    vector_system_6d(vector_system_3d, system_3d);

    const VectorN force = (VectorN() << 1.0, -0.5, 2.0, -2.0, -1.0, 0.0).finished();

    // Step all of the models
    for(unsigned i = 0; i < 5; ++i){
        system_1d.Step(force.head<1>());
        system_2d.Step(force.block<2,1>(1,0));
        system_3d.Step(force.tail<3>());
        vector_system_3d.Step(force.head<3>());
        vector_system_6d.Step(force);
    }

    // Verify equality of all models and vectors
    const Eigen::Vector3d position_1_and_2 = (Eigen::Vector3d() << system_1d.GetPosition(), system_2d.GetPosition()).finished();
    EXPECT_TRUE(gtfo::IsEqual(position_1_and_2, vector_system_3d.GetPosition()));

    const VectorN position_3_and_3 = (VectorN() << vector_system_3d.GetPosition(), system_3d.GetPosition()).finished();
    EXPECT_TRUE(gtfo::IsEqual(position_3_and_3, vector_system_6d.GetPosition()));

    EXPECT_TRUE(gtfo::IsEqual(system_1d.GetPosition(), vector_system_6d.GetModel<0>().GetModel<0>().GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(system_2d.GetPosition(), vector_system_6d.GetModel<0>().GetModel<1>().GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(system_3d.GetPosition(), vector_system_6d.GetModel<1>().GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(vector_system_3d.GetPosition(), vector_system_6d.GetModel<0>().GetPosition()));
}