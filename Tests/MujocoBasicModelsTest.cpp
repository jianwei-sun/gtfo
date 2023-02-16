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

// Verifies rule-of-5 for MujocoWrapper
TEST(MujocoBasicModelsTest, RuleOfFive)
{
    using Wrapper = gtfo::MujocoWrapper<7>;

    // Regular constructor
    Wrapper w1("arms.xml", 0.001);

    // Copy constructor
    Wrapper w2(w1);

    // Move constructor
    Wrapper w3 = std::move(w1);

    // Assignment operator
    w2 = w1;

    // Move assignment operator
    w3 = std::move(w1);

    EXPECT_TRUE(false);
}