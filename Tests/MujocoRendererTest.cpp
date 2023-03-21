#include <gtest/gtest.h>
#include <iostream>
#include <chrono>

#include "../gtfo.hpp"

TEST(MujocoRendererTest, SmokeTest)
{
    std::cout << "It loaded the test :)" << std::endl;
    using VectorN = gtfo::MujocoModel<7>::VectorN;
    gtfo::MujocoModel<7> mujoco_wrapper("arms.xml", 0.001);
    std::cout << "creating the model" << std::endl;

    mujoco_wrapper.Step(VectorN::Constant(0.1));

    gtfo::MujocoRenderer mujoco_renderer(mujoco_wrapper);

    std::cout << "started the renderer" << std::endl;


    for(unsigned i = 0; i < 1000; ++i){
        mujoco_wrapper.Step(VectorN::Constant(0.1));
        //debug with 1 thread first
        // mujoco_renderer.render();

        std::this_thread::sleep_for(std::chrono::milliseconds(15));

    }
    std::cout << "got past step" << std::endl;

    mujoco_renderer.set_should_render(false);

    EXPECT_TRUE(false);
    

}