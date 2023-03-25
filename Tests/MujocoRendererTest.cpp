#include <gtest/gtest.h>
#include <iostream>
#include <chrono>

#include "../gtfo.hpp"

TEST(MujocoRendererTest, SmokeTest)
{
    using VectorN = gtfo::MujocoModel<7>::VectorN;
    gtfo::MujocoModel<7> mujoco_wrapper("arms.xml", 0.001);
    mujoco_wrapper.Step(VectorN::Constant(0.1));

    gtfo::MujocoRenderer mujoco_renderer(mujoco_wrapper);

    for(unsigned i = 0; i < 1000; ++i){
        mujoco_wrapper.Step(VectorN::Constant(0.1));
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    mujoco_renderer.set_should_render(false);

    EXPECT_TRUE(false); //
}