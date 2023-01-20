#include <gtest/gtest.h>

#include "../gtfo.hpp"

// Verifies that an unbounded point mass with second order dynamics moves without stopping
TEST(SecondOrderDynamicsTest, MoveUpwards)
{
    const unsigned dimensions = 3;

    // Create the gtfo object given the type of model you want
    gtfo::PointMassSecondOrder<dimensions, float> test_admittance_controller;

    // Declare Parameters of the model
    const float cycle_time_step = 2e-3f; // 2ms
    const float mass = 10.0f;
    const float damping = 15.0f;
    gtfo::SecondOrderParameters<float> test_parameters(cycle_time_step, mass, damping);

    // Set those parameters for the model
    test_admittance_controller.SetParameters(test_parameters);

    // Define fake force input for force sensor
    Eigen::Matrix<float, dimensions, 1> force_input;
    force_input << 0, 0, 20;

    // Position and velocity
    Eigen::Matrix<float, dimensions, 1>
        position;
    Eigen::Matrix<float, dimensions, 1> velocity;

    for(unsigned i = 0; i < 20000; ++i)
    {
        position = test_admittance_controller.GetPosition();
        velocity = test_admittance_controller.GetVelocity();
        std::cout << "Pos: " << position.transpose() << ", vel: " << velocity.transpose() << "\n";
        test_admittance_controller.Step(force_input);
    }

    EXPECT_TRUE(gtfo::IsEqual(velocity, Eigen::Matrix<float, dimensions, 1>(0, 0, 1.3333)));
}
