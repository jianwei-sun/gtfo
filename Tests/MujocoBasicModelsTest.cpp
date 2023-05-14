#include <gtest/gtest.h>
#include "gtfo.hpp"

// Verifies that MujocoModel can be built
TEST(MujocoBasicModelsTest, DataTypes)
{
    gtfo::MujocoModel<7, double> system_double("arms.xml", 1, 0.001);
    gtfo::MujocoModel<7, float> system_float("arms.xml", 1, 0.001f);

    for(unsigned i = 0; i < 10; ++i){
        system_double.Step(Eigen::Matrix<double, 7, 1>::Zero());
        system_float.Step(Eigen::Matrix<float, 7, 1>::Zero());
    }

    EXPECT_TRUE(gtfo::IsEqual(system_double.GetPosition(), Eigen::Matrix<double, 7, 1>::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system_float.GetPosition(), Eigen::Matrix<float, 7, 1>::Zero()));
}

// Verifies that a model can be loaded and run
TEST(MujocoBasicModelsTest, LoadArmsModel)
{
    using VectorN = gtfo::MujocoModel<7>::VectorN;
    gtfo::MujocoModel<7> system("arms.xml", 1, 0.001);

    for(unsigned i = 0; i < 1000; ++i){
        system.Step(VectorN::Zero());
    }

    std::cout << "Final position: " << system.GetPosition().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), VectorN::Zero()));

    std::cout << "Final velocity: " << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), VectorN::Zero()));

    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));
}

// Verifies rule-of-5 for MujocoModel and that there are no segfaults
TEST(MujocoBasicModelsTest, RuleOfFive)
{
    using Wrapper = gtfo::MujocoModel<7>;

    // Regular constructor
    Wrapper w1("arms.xml", 1, 0.001);

    // Copy constructor
    Wrapper w2(w1);

    // Move constructor
    Wrapper w3_temp("arms.xml", 1, 0.001);
    Wrapper w3 = std::move(w3_temp);

    // Assignment operator
    w2 = w1;

    // Move assignment operator
    w3 = std::move(w1);

    EXPECT_TRUE(true);
}

// Verifies that pausing works
TEST(MujocoBasicModelsTest, PauseDynamics)
{
    using VectorN = gtfo::MujocoModel<7>::VectorN;

    const VectorN initial_position = VectorN::Constant(0.1);

    gtfo::MujocoModel<7> system("arms.xml", 1, 0.001, initial_position);

    system.PauseDynamics(true);
    for(unsigned i = 0; i < 1000; ++i){
        system.Step(VectorN::Ones());
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), initial_position));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), VectorN::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));

    system.PauseDynamics(false);
    for(unsigned i = 0; i < 1000; ++i){
        system.Step(VectorN::Ones());
    }

    EXPECT_FALSE(gtfo::IsEqual(system.GetPosition(), VectorN::Zero()));
    EXPECT_FALSE(gtfo::IsEqual(system.GetPosition(), initial_position));
    EXPECT_FALSE(gtfo::IsEqual(system.GetVelocity(), VectorN::Zero()));
    EXPECT_FALSE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));
}

// Verify hard bound works with Mujoco model
TEST(MujocoBasicModelsTest, HardBoundTest)
{
    using VectorN = gtfo::MujocoModel<7>::VectorN;

    const VectorN initial_position = VectorN::Constant(0.1);

    gtfo::MujocoModel<7> system("arms.xml", 1, 0.001, initial_position);
    system.SetHardBound(gtfo::RectangleBound<7>(VectorN::Constant(0.1)));

    system.PauseDynamics(true);
    for(unsigned i = 0; i < 1000; ++i){
        system.Step(VectorN::Constant(1.0));
    }

    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), VectorN::Constant(0.1)));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), VectorN::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), VectorN::Zero()));
}