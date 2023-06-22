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
    constexpr unsigned state_dimension = 3;
    constexpr unsigned constraint_dimension = 1;
    const int trials = 1000;

    // Vector Types
    using VectorN = Eigen::Matrix<double, state_dimension, 1>;
    using VectorK = Eigen::Matrix<double, constraint_dimension, 1>;
    using MatrixKN = Eigen::Matrix<double, constraint_dimension, state_dimension>;
    using MatrixNN = Eigen::Matrix<double, state_dimension, state_dimension>;
    using MatrixK2K = Eigen::Matrix<double, constraint_dimension, 2*constraint_dimension>;

    // System Parameters
    const double mass = 1.0;
    const double damping = 1.0;   
    const double cycle_time_step = 0.1;

    const double wn = 5;
    const double zeta = 1;
    const MatrixK2K transversal_gain = (MatrixK2K() << wn*wn, 2*zeta*wn).finished(); // can be tuned
    const Eigen::Vector3d initial_position(0, 0, 1); // start at z=1, should move to z=0
    const VectorN initial_velocity = VectorN::Zero();

    // Define System
    gtfo::PointMassSecondOrder<state_dimension> system((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    
    // Define constraint surface
    // Here we use the XY plane, so constraint is the Z value
    gtfo::ManifoldConstraints<state_dimension, constraint_dimension> surface_constraint;

    // h(X) = X[2] = z
    const std::function<VectorK(const VectorN&)> constraint_function = [](const VectorN& position){
        return (VectorK() << position[2]).finished();
    };

    // dh/dX = [0,0,1]
    const std::function<MatrixKN(const VectorN&)> constraint_function_gradient = [](const VectorN& position){
        return (MatrixKN() << 0, 0, 1).finished();
    };

    // d2h/dX2 = 0
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice = [](const VectorN& position){
        return MatrixNN::Zero();
    };
    const std::array< std::function<MatrixNN(const VectorN&)> , 1> constraint_function_hessian_slices = {constraint_function_hessian_slice};
    surface_constraint.SetConstraintFunction(constraint_function, constraint_function_gradient, constraint_function_hessian_slices);
    surface_constraint.SetTransversalGain(transversal_gain);
    
    // Define system dynamics for constraints
    // X_dot = f(X) + g(X)*u
    const auto f_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return -damping / mass * velocity; // VectorN output
    };
    const auto g_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return 1/mass * MatrixNN::Identity(); // MatrixNN output
    };
    surface_constraint.SetSecondOrderDynamics(f_bottom_half, g_bottom_half);

    system.SetForcePremodifier([&](const VectorN& force, const gtfo::DynamicsBase<state_dimension>& system){
       return surface_constraint.Step(force, system.GetPosition(), system.GetVelocity());
    });
    system.SetPositionAndVelocity(initial_position, initial_velocity);

    // Constraint is z=0, so step multiple iterations and then check if z has gotten close to zero
    for(unsigned i = 0; i < trials; ++i){
        system.Step(Eigen::Vector3d::Ones());
        auto position = system.GetPosition();
    }
    std::cout<<system.GetPosition().transpose()<<std::endl;
    EXPECT_TRUE(gtfo::IsEqual(VectorK(system.GetPosition()[2]), VectorK::Zero()));

    // System with no constraint for comparison:
    gtfo::PointMassSecondOrder<state_dimension> system_no_constraint((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    system_no_constraint.SetPositionAndVelocity(initial_position, initial_velocity);
    for(unsigned i = 0; i < trials; ++i){
        system_no_constraint.Step(Eigen::Vector3d::Ones());
    }

    // Check if x and y match system with no constraint (constraint should only affect z)
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition().segment<2>(0), system_no_constraint.GetPosition().segment<2>(0)));

}