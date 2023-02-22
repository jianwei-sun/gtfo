#include <gtest/gtest.h>

#include "../gtfo.hpp"

TEST(MujocoRendererTest, SmokeTest)
{
    using VectorN = gtfo::MujocoWrapper<7>::VectorN;
    gtfo::MujocoWrapper<7> mujoco_wrapper("arms.xml", 0.001);

    gtfo::MujocoRenderer mujoco_renderer(mujoco_wrapper);

    for(unsigned i = 0; i < 1000; ++i){
        mujoco_wrapper.Step(VectorN::Zero());
    }

    

}