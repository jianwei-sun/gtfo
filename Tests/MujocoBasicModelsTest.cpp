#include <gtest/gtest.h>

#include "../gtfo.hpp"

// Verifies that a model can be loaded and run
TEST(MujocoBasicModelsTest, LoadArmsModel)
{
    using VectorN = gtfo::MujocoWrapper<7>::VectorN;
    gtfo::MujocoWrapper<7> mujoco_wrapper("arms.xml", 0.001);

    for(unsigned i = 0; i < 1000; ++i){
        mujoco_wrapper.Step(VectorN::Zero());
    }

    std::cout << "Final position: " << mujoco_wrapper.GetPosition().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(mujoco_wrapper.GetPosition(), VectorN::Zero()));

    std::cout << "Final velocity: " << mujoco_wrapper.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(mujoco_wrapper.GetVelocity(), VectorN::Zero()));

    std::cout << "Final acceleration: " << mujoco_wrapper.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(mujoco_wrapper.GetAcceleration(), VectorN::Zero()));
}

// Verifies rule-of-5 for MujocoWrapper and that there are no segfaults
TEST(MujocoBasicModelsTest, RuleOfFive)
{
    using Wrapper = gtfo::MujocoWrapper<7>;

    // Regular constructor
    Wrapper w1("arms.xml", 0.001);

    // Copy constructor
    Wrapper w2(w1);

    // Move constructor
    Wrapper w3_temp("arms.xml", 0.001);
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
    using VectorN = gtfo::MujocoWrapper<7>::VectorN;

    const VectorN initial_position = VectorN::Constant(0.1);

    gtfo::MujocoWrapper<7> mujoco_wrapper("arms.xml", 0.001, initial_position);

    mujoco_wrapper.PauseDynamics(true);
    for(unsigned i = 0; i < 1000; ++i){
        mujoco_wrapper.Step(VectorN::Ones());
    }

    EXPECT_TRUE(gtfo::IsEqual(mujoco_wrapper.GetPosition(), initial_position));
    EXPECT_TRUE(gtfo::IsEqual(mujoco_wrapper.GetVelocity(), VectorN::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(mujoco_wrapper.GetAcceleration(), VectorN::Zero()));

    mujoco_wrapper.PauseDynamics(false);
    for(unsigned i = 0; i < 1000; ++i){
        mujoco_wrapper.Step(VectorN::Ones());
    }

    EXPECT_FALSE(gtfo::IsEqual(mujoco_wrapper.GetPosition(), VectorN::Zero()));
    EXPECT_FALSE(gtfo::IsEqual(mujoco_wrapper.GetPosition(), initial_position));
    EXPECT_FALSE(gtfo::IsEqual(mujoco_wrapper.GetVelocity(), VectorN::Zero()));
    EXPECT_FALSE(gtfo::IsEqual(mujoco_wrapper.GetAcceleration(), VectorN::Zero()));
}