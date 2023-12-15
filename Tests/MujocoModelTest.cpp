#include <gtest/gtest.h>
#include "gtfo.hpp"

// Verifies that MuJoCo is compiled and linked correctly
TEST(CMakeSmokeTest, MuJoCoVersion){
  EXPECT_TRUE(mjVERSION_HEADER == mj_version());
  EXPECT_EQ(mjVERSION_HEADER, 231);
}

// Verifies that MujocoModel can be built
TEST(MujocoModelTest, DataTypes)
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
TEST(MujocoModelTest, LoadArmsModel)
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
TEST(MujocoModelTest, RuleOfFive)
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
TEST(MujocoModelTest, PauseDynamics)
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
TEST(MujocoModelTest, HardBoundTest)
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

// Verify the rigid body dynamics with MuJoCo
TEST(MujocoModelTest, RigidBodyDynamicsComparison)
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