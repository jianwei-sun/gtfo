#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(ManifoldConstraintsTest, PassThroughTest)
{
    gtfo::ManifoldConstraints<4, 2> manifold_constraints;

    for(unsigned i = 0; i < 10; ++i){
        Eigen::Vector4d force = Eigen::Vector4d::Ones() * i;
        EXPECT_TRUE(gtfo::IsEqual(manifold_constraints.Step(force, Eigen::Vector4d::Ones(), Eigen::Vector4d::Ones()) , force));
    }

}

TEST(ManifoldConstraintsTest, SurfaceTest)
{
    // Create a second order system that has different parameters in the coordinates
    const gtfo::SecondOrderParameters<double> parameters_1(0.1, 0.5, 0.5);
    const gtfo::SecondOrderParameters<double> parameters_2(0.1, 5.0, 5.0);

    gtfo::DynamicsVector<
        gtfo::PointMassSecondOrder<2>,
        gtfo::PointMassSecondOrder<2>>
    system(
        gtfo::DynamicsVector<gtfo::PointMassSecondOrder<2>, gtfo::PointMassSecondOrder<2>>(
            parameters_1, parameters_2
        )
    );

    // Create a second system to act as a control
    auto system_unconstrained = system;

    // Move the system to some aribitrary point in space
    for(unsigned i = 0; i < 10; ++i){
        const Eigen::Vector4d input(1.0, -0.5, -1.0, 0.5);
        system.Step(input);
        system_unconstrained.Step(input);
    }

    // Construct the manifold constraints, where the constraint function is just the inner 
    // two coordinates: h(x1, x2, x3, x4) = (x2, x3)
    gtfo::ManifoldConstraints<4, 2> manifold_constraints;

    // Set the second order dynamics
    const Eigen::Matrix4d mass_inverse = (Eigen::Vector4d() << 
        parameters_1.mass, parameters_1.mass, parameters_2.mass, parameters_2.mass)
        .finished().cwiseInverse().asDiagonal();
        
    manifold_constraints.SetSecondOrderDynamics(
        // f bottom half
        [&, mass_inverse](const Eigen::Vector4d& position, const Eigen::Vector4d& velocity){
            return -mass_inverse * (Eigen::Vector4d() << 
                parameters_1.damping, parameters_1.damping, parameters_2.damping, parameters_2.damping)
                .finished().asDiagonal() * velocity;
        },
        // g bottom half
        [mass_inverse](const Eigen::Vector4d& position, const Eigen::Vector4d& velocity){
            return mass_inverse;
        }
    );

    // SetConstraintFunction
    const std::function<Eigen::Vector2d(const Eigen::Vector4d&)> constraint_function[](const Eigen::Vector4d& position){
        return (Eigen::Vector2d() << position[1], position[2]).finished();
    }

    // dh/dX = [0,1,0,0; 0, 0, 1, 0]
    const std::function<Eigen::Matrix<double, 2, 4>(const Eigen::Vector4d&)> constraint_function_gradient[](const Eigen::Vector4d& position){
        return (Eigen::Matrix<double, 2, 4>() << 0, 1, 0, 0, 0, 0, 1, 0).finished();
    }

    // d2h/dX2 = 0 for both slices
    const std::function<Eigen::Matrix4d(const Eigen::Vector4d&)> constraint_function_hessian_slice_1[](const Eigen::Vector4d& position){
        return Eigen::Matrix4d::Zero();
    }
    const std::array< std::function<Eigen::Matrix4d(const Eigen::Vector4d&)> , 2> constraint_function_hessian_slices = {constraint_function_hessian_slice_1, constraint_function_hessian_slice_1};
    
    // set constraint functions
    manifold_constraints.SetConstraintFunction(constraint_function, constraint_function_gradient, constraint_function_hessian_slices)
    
    // SetTransversalGain
    manifold_constraints.SetTranversalGain(Eigen::RowVector2d(0.1, 0.1));
    
    // Associate the manifold constraints with the virtual system
    system.SetForcePremodifier([&](const Eigen::Vector4d& force, const gtfo::DynamicsBase<4>& system){
       return manifold_constraints.Step(force, system.GetPosition(), system.GetVelocity());
    });

    // Step the systems again, which now includes the constraint manifold
    for(unsigned i = 0; i < 10; ++i){
        const Eigen::Vector4d input(1.0, -0.5, -1.0, 0.5);
        system.Step(input);
        system_unconstrained.Step(input);
    }

    // Verify that the constrained coordinates are 0
    EXPECT_FALSE(gtfo::IsEqual(system_unconstrained.GetPosition().segment<2>(1), Eigen::Vector2d::Zero()));
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition().segment<2>(1), Eigen::Vector2d::Zero()));

    // Verify that the unconstrained coordinates are unaffected
    EXPECT_TRUE(gtfo::IsEqual(
        Eigen::Vector2d(system.GetPosition()[0], system.GetPosition()[3]),
        Eigen::Vector2d(system_unconstrained.GetPosition()[0], system_unconstrained.GetPosition()[3])
    ));

    std::cout << system.GetPosition().transpose() << std::endl; 
}