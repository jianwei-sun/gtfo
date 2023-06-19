#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(ForcePremodifierTest, FlipForces)
{
    gtfo::PointMassSecondOrder<2> system((gtfo::SecondOrderParameters<double>()));

    auto lambda = [](const Eigen::Vector2d& force, const gtfo::DynamicsBase<2>& system){
        return -force;
    };
    system.SetForcePremodifier(lambda);

    for(unsigned i = 0; i < 100; ++i){
        system.Step(Eigen::Vector2d::Ones());
    }
    EXPECT_TRUE((system.GetPosition().array() < 0.0).all());
}


TEST(ForcePremodifierTest, Containers)
{
    gtfo::DynamicsSelector<
        gtfo::DynamicsVector<
            gtfo::PointMassSecondOrder<2>,
            gtfo::PointMassFirstOrder<2>>,
        gtfo::PointMassSecondOrder<4>>
    system(
        gtfo::DynamicsVector<gtfo::PointMassSecondOrder<2>,gtfo::PointMassFirstOrder<2>>(
            (gtfo::SecondOrderParameters<double>()), (gtfo::FirstOrderParameters<double>())), 
        (gtfo::SecondOrderParameters<double>()));

    // Make the first system just pass in the opposite of the force in the first two coordinates
    system.GetModel<0>().SetForcePremodifier([](const Eigen::Vector4d& force, const gtfo::DynamicsBase<4>& system){
       return force.cwiseProduct(Eigen::Vector4d(-1.0, -1.0, 1.0, 1.0)).eval();
    });

    // Make the second system be a PD controller
    system.GetModel<1>().SetForcePremodifier([](const Eigen::Vector4d& force, const gtfo::DynamicsBase<4>& system){
       return -1.0 * system.GetPosition() - 0.5 * system.GetVelocity();
    });

    // Step the first system
    for(unsigned i = 0; i < 10; ++i){
        system.Step(Eigen::Vector4d::Ones());
    }

    // Ensure that the first second order model behaves correctly
    std::cout << system.GetPosition().block<2,1>(0,0) << std::endl;
    EXPECT_TRUE((system.GetPosition().block<2,1>(0,0).array() < 0.0).all());
    EXPECT_TRUE((system.GetPosition().block<2,1>(2,0).array() > 0.0).all());

    // Note that when switching to the second system, its initial conditions are that of the first system
    const Eigen::Vector4d first_position = system.GetPosition();
    const Eigen::Vector4d first_velocity = system.GetVelocity();
    const Eigen::Vector4d first_acceleration = system.GetAcceleration();
    EXPECT_TRUE(system.Select(1));
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), first_position));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), first_velocity));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), first_acceleration));

    // Now step the second system
    for(unsigned i = 0; i < 20; ++i){
        system.Step(Eigen::Vector4d::Ones());
    }

    // Verify that the second system has stabilized to the origin
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector4d::Zero()));
}

TEST(ForcePremodifierTest, XYSurfaceConstraint)
{
    const unsigned state_dimension = 3;
    const unsigned constraint_dimension = 1;

    // Vector Types
    using VectorN = Eigen::Matrix<float, state_dimension, 1>;
    using VectorK = Eigen::Matrix<float, constraint_dimension, 1>;
    using MatrixKN = Eigen::Matrix<float, constraint_dimension, state_dimension>;
    using MatrixNN = Eigen::Matrix<float, state_dimension, state_dimension>;
    using MatrixK2K = Eigen::Matrix<float, constraint_dimension, 2*constraint_dimention>;
    
    // Define System
    gtfo::PointMassSecondOrder<state_dimension> system((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));

    // System Parameters
    const float mass = 1.0;
    const float damping = 1.0;   
    const float cycle_time_step = 0.1;
    const MatrixK2K transversal_gain = (MatrixK2K() << 0.1, 0.1).finished(); // can be tuned
    const VectorN initial_position = (VectorN() << 0, 0, 1).finished(); // start at z=1, should move to z=0
    const VectorN initial_velocity = VectorN::Zero();
    
    // Define constraint surface
    // Here we use the XY plane, so constraint is the Z value
    ManifoldConstraints surface_constraint;

    // h(X) = X[2] = z
    VectorK constraint_function[](const VectorN& position){
        return (VectorK() << position[2]).finished();
    }

    // dh/dX = [0,0,1]
    MatrixKN constraint_function_gradient[](const VectorN& position){
        return (MatrixKN() << 0, 0, 1).finished();
    }

    // d2h/dX2 = 0
    MatrixNN constraint_function_hessian_slice[](const VectorN& position){
        return MatrixNN::Zero();
    }
    surface_constraint.SetConstraintFunction(constraint_function, constraint_function_gradient, constraint_function_hessian_slice)
    surface_constraint.SetTranversalGain(transversal_gain);
    
    // Define system dynamics for constraints
    // X_dot = f(X) + g(X)*u
    VectorN f_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return -damping / mass * velocity;
    };
    MatrixNN g_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return 1/mass * MatrixNN::Identity();
    };
    surface_constraint.SetSecondOrderDynamics(f_bottom_half, g_bottom_half);

    system.SetForcePremodifier(surface_constraint);
    system.SetPositionAndVelocity(initial_position, initial_velocity);

    // Constraint is z=0, so step multiple iterations and then check if z has gotten close to zero
    for(unsigned i = 0; i < 100; ++i){
        system.Step(Eigen::Vector2d::Ones());
    }
    EXPECT_TRUE((std::abs(system.GetPosition()[2]) < 0.1));
}