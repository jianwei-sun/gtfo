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
    force_input << 0.0f, 0.0f, 20.0f;

    // Position and velocity
    Eigen::Matrix<float, dimensions, 1>
        position;
    Eigen::Matrix<float, dimensions, 1> velocity;

    for(unsigned i = 0; i < 20000; ++i)
    {
        test_admittance_controller.Step(force_input);
    }

    std::cout << "Final position: " << test_admittance_controller.GetPosition().transpose() << "\n";
    std::cout << "Final velocity: " << test_admittance_controller.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(test_admittance_controller.GetVelocity(), Eigen::Matrix<float, dimensions, 1>(0.0f, 0.0f, 1.3333f)));
}
