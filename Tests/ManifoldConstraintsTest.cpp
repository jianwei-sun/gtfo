#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(ManifoldConstraintsTest, PassThroughTest)
{
    gtfo::ManifoldConstraints<4, 2> manifold_constraints;

    for(unsigned i = 0; i < 10; ++i){
        Eigen::Vector4d force = Eigen::Vector4d::Ones() * i;
        EXPECT_TRUE(gtfo::IsEqual(manifold_constraints.Step(force, Eigen::Vector4d::Ones(), Eigen::Vector4d::Ones()), force));
    }

}

TEST(ManifoldConstraintsTest, SurfaceTest)
{
    // Create a second order system that has different parameters in the coordinates
    static const gtfo::SecondOrderParameters<double> parameters_1(0.1, 0.5, 0.5);
    static const gtfo::SecondOrderParameters<double> parameters_2(0.1, 5.0, 5.0);

    constexpr double wn = 5;
    constexpr double zeta = 1;
    static const Eigen::RowVector2d transversal_gain(wn*wn, 2*zeta*wn); // can be tuned
    constexpr int trials = 1000;

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
    const std::function<Eigen::Vector2d(const Eigen::Vector4d&)> constraint_function = [](const Eigen::Vector4d& position){
        return (Eigen::Vector2d() << position[1], position[2]).finished();
    };

    // dh/dX = [0,1,0,0; 0, 0, 1, 0]
    const std::function<Eigen::Matrix<double, 2, 4>(const Eigen::Vector4d&)> constraint_function_gradient = [](const Eigen::Vector4d& position){
        return (Eigen::Matrix<double, 2, 4>() << 0, 1, 0, 0, 0, 0, 1, 0).finished();
    };

    // d2h/dX2 = 0 for both slices
    const std::function<Eigen::Matrix4d(const Eigen::Vector4d&)> constraint_function_hessian_slice_1 = [](const Eigen::Vector4d& position){
        return Eigen::Matrix4d::Zero();
    };
    const std::array< std::function<Eigen::Matrix4d(const Eigen::Vector4d&)> , 2> constraint_function_hessian_slices = {constraint_function_hessian_slice_1, constraint_function_hessian_slice_1};
    
    // set constraint functions
    manifold_constraints.SetConstraintFunction(constraint_function, constraint_function_gradient, constraint_function_hessian_slices);
    
    // SetTransversalGain
    manifold_constraints.SetTransversalGain(transversal_gain);
    
    // Associate the manifold constraints with the virtual system
    system.SetForcePremodifier([&](const Eigen::Vector4d& force, const gtfo::DynamicsBase<4>& system){
       return manifold_constraints.Step(force, system.GetPosition(), system.GetVelocity());
    });

    // Step the systems again, which now includes the constraint manifold
    for(unsigned i = 0; i < trials; ++i){
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

TEST(ManifoldConstraintsTest, XYSurfaceConstraint)
{
    constexpr unsigned state_dimension = 3;
    constexpr unsigned constraint_dimension = 1;
    constexpr int trials = 1000;

    // Vector Types
    using VectorN = Eigen::Matrix<double, state_dimension, 1>;
    using VectorK = Eigen::Matrix<double, constraint_dimension, 1>;
    using MatrixKN = Eigen::Matrix<double, constraint_dimension, state_dimension>;
    using MatrixNN = Eigen::Matrix<double, state_dimension, state_dimension>;
    using MatrixK2K = Eigen::Matrix<double, constraint_dimension, 2*constraint_dimension>;

    // System Parameters
    constexpr double mass = 1.0;
    constexpr double damping = 1.0;   
    constexpr double cycle_time_step = 0.1;

    constexpr double wn = 5;
    constexpr double zeta = 1;
    static const MatrixK2K transversal_gain = (MatrixK2K() << wn*wn, 2*zeta*wn).finished(); // can be tuned
    static const Eigen::Vector3d initial_position(0, 0, 1); // start at z=1, should move to z=0
    static const VectorN initial_velocity = VectorN::Zero();

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
    system.SetState(initial_position, initial_velocity);

    // Constraint is z=0, so step multiple iterations and then check if z has gotten close to zero
    for(unsigned i = 0; i < trials; ++i){
        system.Step(Eigen::Vector3d::Ones());
    }
    EXPECT_TRUE(gtfo::IsEqual(VectorK(system.GetPosition()[2]), VectorK::Zero()));

    // System with no constraint for comparison:
    gtfo::PointMassSecondOrder<state_dimension> system_no_constraint((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    system_no_constraint.SetState(initial_position, initial_velocity);
    for(unsigned i = 0; i < trials; ++i){
        system_no_constraint.Step(Eigen::Vector3d::Ones());
    }

    // Check if x and y match system with no constraint (constraint should only affect z)
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition().segment<2>(0), system_no_constraint.GetPosition().segment<2>(0)));

}

TEST(ManifoldConstraintsTest, CircularPathConstraint)
{
    constexpr unsigned state_dimension = 3;
    constexpr unsigned constraint_dimension = 2;
    constexpr unsigned radius = 1;
    const unsigned trials = 1000;

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
    const Eigen::RowVector2d transversal_gain(wn*wn, 2*zeta*wn);
    const Eigen::Vector3d initial_position(0.5, 0, 1); // start at z=1, x=0.5, should move to z=0 x=1
    const VectorN initial_velocity = VectorN::Zero();

    // Define System
    gtfo::PointMassSecondOrder<state_dimension> system((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    
    // Define constraint surface
    // Here we use the XY plane, so constraint is the Z value
    gtfo::ManifoldConstraints<state_dimension, constraint_dimension> surface_constraint;

    // h(X) = [sqrt(x^2 + y^2) - r; z]
    const std::function<VectorK(const VectorN&)> constraint_function = [radius](const VectorN& position){
        return VectorK(position.head<2>().norm()  - radius, position[2]);
    };

    // dh/dX
    const std::function<MatrixKN(const VectorN&)> constraint_function_gradient = [](const VectorN& position){
        return (MatrixKN() << position.head<2>().normalized().transpose(), 0.0, VectorN::UnitZ().transpose()).finished();
    };

    // d2h/dX2
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice_1 = [](const VectorN& position){
        MatrixNN output = MatrixNN::Zero();
        const double xy_norm = position.head<2>().norm();
        const double xy_norm_cubed = std::pow(xy_norm, 3);
        output.topLeftCorner<2,2>() = Eigen::Matrix2d::Identity() / xy_norm - position.head<2>() * position.head<2>().transpose() / std::pow(xy_norm, 3);
        return output;
    };
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice_2 = [](const VectorN& position){
        return MatrixNN::Zero();
    };

    const std::array< std::function<MatrixNN(const VectorN&)> , 2> constraint_function_hessian_slices = {constraint_function_hessian_slice_1, constraint_function_hessian_slice_2};
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
    system.SetState(initial_position, initial_velocity);

    // Step multiple iterations with external force and check if point goes to, and remains on, the circular path
    for(unsigned i = 0; i < trials; ++i){
        system.Step(Eigen::Vector3d::Ones());
    }
    std::cout << system.GetPosition()<< std::endl;

    // Check if x,y values lie on the circle
    EXPECT_NEAR(system.GetPosition().head<2>().norm(), radius, 0.01);
    // Check if z is zero
    EXPECT_NEAR(system.GetPosition()[2], 0.0, 0.01);

}


TEST(ManifoldConstraintsTest, EllipticalPathConstraint)
{
    constexpr unsigned state_dimension = 3;
    constexpr unsigned constraint_dimension = 2;
    constexpr double a = 1;
    constexpr double b = 2;
    constexpr double x0 = 0;
    constexpr double y0 = 0;
    constexpr unsigned trials = 1000;

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

    const double wn = 0.5;
    const double zeta = 0.7;
    static const Eigen::RowVector2d transversal_gain(wn*wn, 2*zeta*wn);
    static const Eigen::Vector3d initial_position(0, 0, 1); // start at z=1, x=0.5, should move to z=0 x=1
    static const VectorN initial_velocity = VectorN::Zero();

    // Define System
    gtfo::PointMassSecondOrder<state_dimension> system((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    
    // Define constraint surface
    // Here we use the XY plane, so constraint is the Z value
    gtfo::ManifoldConstraints<state_dimension, constraint_dimension> surface_constraint;

    // h(X) = [(x-x0)^2/a^2 + (y-y0)^2/b^2 - 1; z]
    const std::function<VectorK(const VectorN&)> constraint_function = [a, b, x0, y0](const VectorN& position){
        return VectorK(std::pow(position[0] - x0, 2)/a/a + std::pow(position[1] - y0, 2)/b/b - 1, position[2]);
    };

    // dh/dX
    const std::function<MatrixKN(const VectorN&)> constraint_function_gradient = [a, b, x0, y0](const VectorN& position){
        return (MatrixKN() << 2/a/a*(position[0] - x0), 2/b/b*(position[1] - y0), 0.0, VectorN::UnitZ().transpose()).finished();
    };

    // d2h/dX2
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice_1 = [a, b](const VectorN& position){
        return (MatrixNN() << 2/a/a, 0, 0, 0, 2/b/b, 0, 0, 0, 0).finished();
    };
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice_2 = [](const VectorN& position){
        return MatrixNN::Zero();
    };

    const std::array< std::function<MatrixNN(const VectorN&)> , 2> constraint_function_hessian_slices = {constraint_function_hessian_slice_1, constraint_function_hessian_slice_2};
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
    system.SetState(initial_position, initial_velocity);

    // Step multiple iterations with external force and check if point goes to, and remains on, the elliptical path
    for(unsigned i = 0; i < trials; ++i){
        system.Step(Eigen::Vector3d::Ones());
    }
    std::cout << system.GetPosition()<< std::endl;

    // Check if x,y values lie on the ellipse
    EXPECT_NEAR(std::pow(system.GetPosition()[0] - x0, 2)/a/a + std::pow(system.GetPosition()[1] - y0, 2)/b/b, 1, 0.01);
    // Check if z is zero
    EXPECT_NEAR(system.GetPosition()[2], 0.0, 0.01);
}

TEST(ManifoldConstraintsTest, RotatedEllipticalPathConstraint)
{
    constexpr unsigned state_dimension = 3;
    constexpr unsigned constraint_dimension = 2;
    constexpr double a = 1;
    constexpr double b = 2;
    constexpr double x0 = 0;
    constexpr double y0 = 0;
    constexpr unsigned trials = 1000;
    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(0.25*M_PI,  Eigen::Vector3d::UnitY()).toRotationMatrix(); // rotate 45deg about y axis

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

    const double wn = 0.5;
    const double zeta = 0.7;
    static const Eigen::RowVector2d transversal_gain(wn*wn, 2*zeta*wn);
    static const Eigen::Vector3d initial_position(0, 0, 1);
    static const VectorN initial_velocity = VectorN::Zero();

    // Define System
    gtfo::PointMassSecondOrder<state_dimension> system((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    
    // Define constraint surface
    // Here we use the XY plane, so constraint is the Z value
    gtfo::ManifoldConstraints<state_dimension, constraint_dimension> surface_constraint;

    // h(X) = [(x-x0)^2/a^2 + (y-y0)^2/b^2 - 1; z]
    const std::function<VectorK(const VectorN&)> constraint_function = [a, b, x0, y0, rotation_matrix](const VectorN& position){
        VectorN transformed_position = rotation_matrix.inverse() * position;
        return VectorK(std::pow(transformed_position[0] - x0, 2)/a/a + std::pow(transformed_position[1] - y0, 2)/b/b - 1, transformed_position[2]);
    };

    // dh/dX
    const std::function<MatrixKN(const VectorN&)> constraint_function_gradient = [a, b, x0, y0, rotation_matrix](const VectorN& position){
        VectorN transformed_position = rotation_matrix.inverse() * position;
        return MatrixKN((((MatrixKN() << 2/a/a*(transformed_position[0] - x0), 2/b/b*(transformed_position[1] - y0), 0.0, VectorN::UnitZ().transpose()).finished()) * rotation_matrix.inverse()));
    };

    // d2h/dX2
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice_1 = [a, b, rotation_matrix](const VectorN& position){
        return MatrixNN(rotation_matrix * (MatrixNN() << 2/a/a, 0, 0, 0, 2/b/b, 0, 0, 0, 0).finished() * rotation_matrix.inverse());
    };
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice_2 = [](const VectorN& position){
        return MatrixNN::Zero();
    };

    const std::array< std::function<MatrixNN(const VectorN&)> , 2> constraint_function_hessian_slices = {constraint_function_hessian_slice_1, constraint_function_hessian_slice_2};
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

    // Step multiple iterations with external force and check if point goes to, and remains on, the elliptical path
    for(unsigned i = 0; i < trials; ++i){
        system.Step(Eigen::Vector3d::Ones());
        std::cout << "Iteration: "<< i << "..." << std::endl << system.GetPosition().transpose()<< std::endl;
    }
    std::cout << system.GetPosition()<< std::endl;

    // Check if x,y values lie on the ellipse
    VectorN transformed_position = rotation_matrix * system.GetPosition();
    EXPECT_NEAR(std::pow(transformed_position[0] - x0, 2)/a/a + std::pow(transformed_position[1] - y0, 2)/b/b, 1, 0.01);
    // Check if z is zero
    EXPECT_NEAR(transformed_position[2], 0.0, 0.01);
}