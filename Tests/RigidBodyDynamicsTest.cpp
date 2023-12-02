#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "gtfo.hpp"

TEST(RigidBodyDynamicsTest, MujocoComparison)
{   
    using Vector3 = Eigen::Vector3d;
    using Vector6 = Eigen::Matrix<double, 6, 1>;
    using Quaternion = Eigen::Quaterniond;

    // Load and create the MuJoCo model
    EXPECT_TRUE(mjVERSION_HEADER == mj_version());
    std::array<char, 1000> error;
    mjModel* model = mj_loadXML("rectangle.xml", NULL, error.data(), 1000);
    if(!model){
        mju_error("Could not load model from file");
    }
    mjData* data = mj_makeData(model);
    mj_step(model, data);

    // Create the comparable gtfo model
    gtfo::RigidBodySecondOrder<double> rigid_body(
        model->opt.timestep,
        model->body_mass[1],
        Vector3::Map(&model->body_inertia[3]),
        model->dof_damping[0],
        model->dof_damping[3],
        gtfo::Pose<double>(
            Vector3::Map(&data->xpos[3]), 
            Quaternion(data->xquat[4], data->xquat[5], data->xquat[6], data->xquat[7]))
    );

    // Step the models together with the same input
    const Vector6 input = (Vector6() << 0.3, -0.6, 0.0, 0.4, -0.5, 0.1).finished();
    
    Vector6::Map(data->ctrl) = input;
    for(unsigned i = 0; i < 200; ++i){
        mj_step(model, data);
        rigid_body.Step(input);
    }

    // Evaluate that they are near each other
    EXPECT_TRUE(gtfo::IsEqual(Vector3::Map(&data->xpos[3]), rigid_body.GetPose().GetPosition(), 0.002));
    EXPECT_NEAR(Quaternion(data->xquat[4], data->xquat[5], data->xquat[6], data->xquat[7]).angularDistance(rigid_body.GetPose().GetOrientation()), 0.0, M_PI / 36.0);
}