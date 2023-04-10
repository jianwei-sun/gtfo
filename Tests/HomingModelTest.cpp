#include <gtest/gtest.h>
#include "gtfo.hpp"


TEST(HomingModelTest, UnitarySystem)
{
    using Scalar = Eigen::Matrix<double, 1, 1>;
    gtfo::HomingModel<1> system(0.01, 2.0 / 1.8, Scalar(1.0));

    system.ResetHoming(Scalar::Zero());
    
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Scalar::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Scalar::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Scalar::Zero()));

    for(unsigned i = 0; i < 10; ++i){
        system.Step(Scalar::Zero());
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Scalar::Constant(0.0555)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Scalar::Constant(2.0 / 1.8)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Scalar::Constant(20.0 / 1.8)));

    for(unsigned i = 0; i < 40; ++i){
        system.Step(Scalar::Zero());
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Scalar::Constant(0.5)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Scalar::Constant(2.0 / 1.8)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Scalar::Zero()));

    for(unsigned i = 0; i < 50; ++i){
        system.Step(Scalar::Zero());
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Scalar::Constant(1.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Scalar::Constant(0.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Scalar::Zero()));
}

TEST(HomingModelTest, MultidimensionalSystem)
{
    using VectorN = gtfo::HomingModel<3>::VectorN;
    gtfo::HomingModel<3> system(0.001, 0.3);
    
    const VectorN starting_position(2.0, -1.0, 3.0);
    system.ResetHoming(starting_position);

    unsigned int step_counter = 0;

    while(std::abs(system.GetProgress() - 0.5) > GTFO_ALGORITHMIC_CONVERGENCE_TOLERANCE){
        system.Step(VectorN::Zero());
        step_counter++;
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), starting_position / 2.0));
    std::cout << system.GetPosition().transpose() << "\n";
    EXPECT_DOUBLE_EQ(system.GetVelocity().norm(), 0.3);
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));

    while(system.GetProgress() < 1.0){
        system.Step(VectorN::Zero());
        step_counter++;
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), VectorN::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), VectorN::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));

    EXPECT_EQ(step_counter, 13858);
}

TEST(HomingModelTest, DynamicsSelector)
{
    using VectorN = Eigen::Vector2d;
    
    gtfo::DynamicsSelector<
        gtfo::PointMassSecondOrder<2>,
        gtfo::HomingModel<2>>
    system(
        gtfo::PointMassSecondOrder<2>((gtfo::SecondOrderParameters<double>(0.01, 1.0, 1.0))),
        gtfo::HomingModel<2>(0.01, 0.5, VectorN(1.0, 2.0))
    );

    const VectorN force(-3.0, 1.2);

    // Step the second order model first
    for(unsigned i = 0; i < 100; ++i){
        system.Step(force);
    }

    // Store the final position of the second order model
    const VectorN switching_position = system.GetPosition();
    
    // Now switch to homing and verify the starting position is correct
    system.Select(1);
    
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), switching_position));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), VectorN::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));

    // Keep stepping until homing is complete
    while(system.GetModel<1>().GetProgress() < 1.0){
        system.Step(force);
    }

    // Verify the system is indeed stopped at the home position
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), VectorN(1.0, 2.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), VectorN::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));

    // Switch back to the second order system and ensure its position is also the homing position
    system.Select(0);
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), VectorN(1.0, 2.0)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), VectorN::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));
}