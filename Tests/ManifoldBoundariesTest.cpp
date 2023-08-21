#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(ManifoldBoundariesTest, BarrierConstraint1D)
{
    // Vector Types
    using VectorN = Eigen::Matrix<double, 1, 1>;
    using VectorK = Eigen::Matrix<double, 1, 1>;
    using MatrixKN = Eigen::Matrix<double, 1, 1>;
    using MatrixNN = Eigen::Matrix<double, 1, 1>;
    using MatrixK2K = Eigen::Matrix<double, 1, 2>;

    // Define System
    constexpr double mass = 1.0;
    constexpr double damping = 1.0;  
    constexpr double cycle_time_step = 0.01;
    gtfo::PointMassSecondOrder<1> system((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    
    // Define constraint surface
    gtfo::ManifoldConstraints<1, 1> manifold_constraints;

    // Define boundaries and constants
    constexpr double lower = 0.0;
    constexpr double upper = 1.0; 
    constexpr double steepness = 10.0;

    // h(x)
    const std::function<VectorK(const VectorN&)> constraint_function = [=](const VectorN& position){
        const double x = position.value();
        if(x < lower){
            return VectorK(steepness * (lower - x) * (lower - x) * (lower - x));
        } else if(x > upper){
            return VectorK(steepness * (x - upper) * (x - upper) * (x - upper));
        } else{
            return VectorK(0);
        }
    };

    // h'(x)
    const std::function<MatrixKN(const VectorN&)> constraint_function_gradient = [=](const VectorN& position){
        const double x = position.value();
        if(x < lower){
            return MatrixKN(-3.0 * steepness * (lower - x) * (lower - x));
        } else if(x > upper){
            return MatrixKN(3.0 * steepness * (x - upper) * (x - upper));
        } else{
            return MatrixKN(0);
        }
    };

    // h''(x)
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice = [=](const VectorN& position){
        const double x = position.value();
        if(x < lower){
            return MatrixNN(6.0 * steepness * (lower - x));
        } else if(x > upper){
            return MatrixNN(6.0 * steepness * (x - upper));
        } else{
            return MatrixNN(0);
        }
    };
    const std::array<std::function<MatrixNN(const VectorN&)>, 1> constraint_function_hessian_slices{constraint_function_hessian_slice};

    // Set the constraint functions
    manifold_constraints.SetConstraintFunction(constraint_function, constraint_function_gradient, constraint_function_hessian_slices);

    // Set the virtual dynamics
    const auto f_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return -damping / mass * velocity; // VectorN output
    };
    const auto g_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return 1/mass * MatrixNN::Identity(); // MatrixNN output
    };
    manifold_constraints.SetSecondOrderDynamics(f_bottom_half, g_bottom_half);

    // Set the transversal gain
    constexpr double wn = 5.0;
    constexpr double zeta = 1.0;
    static const MatrixK2K transversal_gain = (MatrixK2K() << wn*wn, 2*zeta*wn).finished();
    manifold_constraints.SetTransversalGain(transversal_gain);

    // Associate the manifold constraints with the dynamics
    system.SetForcePremodifier([&](const VectorN& force, const gtfo::DynamicsBase<1>& system){
       const VectorN modified_force = manifold_constraints.Step(force, system.GetPosition(), system.GetVelocity());
       std::cout << modified_force.transpose() << ", ";
       return modified_force;
    });

    // Step the system
    std::cout << "Force, Position, Velocity" << std::endl;
    for(unsigned i = 0; i < 500; ++i){
        system.Step(VectorN(2.0));
        std::cout << system.GetPosition().transpose() << ", " << system.GetVelocity().transpose() << std::endl;
    }

    // Verify the state is in the constraint manifold to some mild tolerance 
    EXPECT_TRUE((lower - 0.1) <= system.GetPosition().value() && system.GetPosition().value() <= (upper + 0.1));
}

TEST(ManifoldBoundariesTest, RectangularBarrierConstraint2D)
{
    // Vector Types
    using VectorN = Eigen::Matrix<double, 2, 1>;
    using VectorK = Eigen::Matrix<double, 2, 1>;
    using MatrixKN = Eigen::Matrix<double, 2, 2>;
    using MatrixNN = Eigen::Matrix<double, 2, 2>;
    using MatrixK2K = Eigen::Matrix<double, 2, 4>;

    // Define System
    constexpr double mass = 1.0;
    constexpr double damping = 1.0;  
    constexpr double cycle_time_step = 0.01;
    gtfo::PointMassSecondOrder<2> system((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    
    // Define constraint surface
    gtfo::ManifoldConstraints<2, 2> manifold_constraints;

    // Define boundaries and constants
    static const VectorK lower(-1.0, -0.5);
    static const VectorK upper(0.5, 1.0);
    constexpr double steepness = 10.0;

    // h(x)
    const std::function<VectorK(const VectorN&)> constraint_function = [&](const VectorN& position){
        return (position.array() < lower.array()).select(steepness * (lower - position).array().pow(3), 
            (position.array() > upper.array()).select(steepness * (position - upper).array().pow(3),
                VectorK::Zero()));
    };

    // h'(x)
    const std::function<MatrixKN(const VectorN&)> constraint_function_gradient = [&](const VectorN& position){
        return MatrixKN(VectorN(
            (position.array() < lower.array()).select(-3.0 * steepness * (lower - position).array().pow(2),
            (position.array() > upper.array()).select(3.0 * steepness * (position - upper).array().pow(2),
                VectorK::Zero()))).asDiagonal());
    };

    // h''(x)
    const std::function<MatrixNN(const VectorN&, const unsigned&)> constraint_function_hessian_slice = [&](const VectorN& position, const unsigned& index){
        const double& x = position[index];
        const double& l = lower[index];
        const double& u = upper[index];
        const double d2h = 
            x < l ? 
                6.0 * steepness * (l - x) : 
            x > u ? 
                6.0 * steepness * (x - u) : 
                0.0;
        if(index == 0){
            return (MatrixNN() << d2h, 0.0, 0.0, 0.0).finished();
        } else{
            return (MatrixNN() << 0.0, 0.0, 0.0, d2h).finished();
        }
    };
    const std::array<std::function<MatrixNN(const VectorN&)>, 2> constraint_function_hessian_slices{
        std::bind(constraint_function_hessian_slice, std::placeholders::_1, 0),
        std::bind(constraint_function_hessian_slice, std::placeholders::_1, 1),
    };

    // Set the constraint functions
    manifold_constraints.SetConstraintFunction(constraint_function, constraint_function_gradient, constraint_function_hessian_slices);

    // Set the virtual dynamics
    const auto f_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return -damping / mass * velocity; // VectorN output
    };
    const auto g_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return 1/mass * MatrixNN::Identity(); // MatrixNN output
    };
    manifold_constraints.SetSecondOrderDynamics(f_bottom_half, g_bottom_half);

    // Set the transversal gain to be the same in both dimensions
    constexpr double wn = 5.0;
    constexpr double zeta = 1.0;
    manifold_constraints.SetTransversalGain(Eigen::RowVector2d(wn * wn, 2.0 * zeta * wn));

    // Associate the manifold constraints with the dynamics
    system.SetForcePremodifier([&](const VectorN& force, const gtfo::DynamicsBase<2>& system){
       const VectorN modified_force = manifold_constraints.Step(force, system.GetPosition(), system.GetVelocity());
       std::cout << modified_force.transpose() << ", ";
       return modified_force;
    });

    // Step the system
    std::cout << "Force, Position, Velocity" << std::endl;
    for(unsigned i = 0; i < 500; ++i){
        system.Step(VectorN(2.0, -1.0));
        std::cout << system.GetPosition().transpose() << ", " << system.GetVelocity().transpose() << std::endl;
    }

    // Verify the state is in the constraint manifold to some mild tolerance 
    EXPECT_TRUE(((lower.array() - 0.1) < system.GetPosition().array()).all() && (system.GetPosition().array() < (upper.array() + 0.1)).all());
}

TEST(ManifoldBoundariesTest, CircularBarrierConstraint2D)
{
    // Vector Types
    using VectorN = Eigen::Matrix<double, 2, 1>;
    using VectorK = Eigen::Matrix<double, 1, 1>;
    using MatrixKN = Eigen::Matrix<double, 1, 2>;
    using MatrixNN = Eigen::Matrix<double, 2, 2>;
    using MatrixK2K = Eigen::Matrix<double, 1, 2>;

    // Define System
    constexpr double mass = 1.0;
    constexpr double damping = 1.0;  
    constexpr double cycle_time_step = 0.01;
    gtfo::PointMassSecondOrder<2> system((gtfo::SecondOrderParameters<double>(cycle_time_step, mass, damping)));
    
    // Define constraint surface
    gtfo::ManifoldConstraints<2, 1> manifold_constraints;

    // Define boundaries and constants
    constexpr double radius = 2.5;
    constexpr double steepness = 10.0;

    // h(x)
    const std::function<VectorK(const VectorN&)> constraint_function = [&](const VectorN& position){
        const double distance = position.norm();
        if(distance >= radius){
            return VectorK(steepness * std::pow(distance - radius, 3));
        } else{
            return VectorK(0.0);
        }
    };

    // h'(x)
    const std::function<MatrixKN(const VectorN&)> constraint_function_gradient = [&](const VectorN& position){
        const double distance = position.norm();
        if(distance >= radius){
            return MatrixKN(3.0 * steepness * std::pow(distance - radius, 2) * position.transpose() / distance);
        } else{
            return MatrixKN(0.0, 0.0);
        }
    };

    // h''(x)
    const std::function<MatrixNN(const VectorN&)> constraint_function_hessian_slice = [=](const VectorN& position){
        const double distance = position.norm();
        if(distance >= radius){
            static const Eigen::Matrix2d transform = (Eigen::Matrix2d() << 0, 1, -1, 0).finished();
            const MatrixKN gradient = position.transpose() / distance;
            return MatrixNN(gradient.transpose() * (6.0 * steepness * (distance - radius)) * gradient + 
            (3.0 * steepness * std::pow(distance - radius, 2)) * transform * position * position.transpose() * transform.transpose() / std::pow(distance, 3));
        } else{
            return (MatrixNN() << 0, 0, 0, 0).finished();
        }
    };
    const std::array<std::function<MatrixNN(const VectorN&)>, 1> constraint_function_hessian_slices{
        constraint_function_hessian_slice
    };

    // Set the constraint functions
    manifold_constraints.SetConstraintFunction(constraint_function, constraint_function_gradient, constraint_function_hessian_slices);

    // Set the virtual dynamics
    const auto f_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return -damping / mass * velocity; // VectorN output
    };
    const auto g_bottom_half = [mass, damping](const VectorN& position, const VectorN& velocity){
        return 1/mass * MatrixNN::Identity(); // MatrixNN output
    };
    manifold_constraints.SetSecondOrderDynamics(f_bottom_half, g_bottom_half);

    // Set the transversal gain 
    constexpr double wn = 5.0;
    constexpr double zeta = 1.0;
    manifold_constraints.SetTransversalGain(Eigen::RowVector2d(wn * wn, 2.0 * zeta * wn));

    // Associate the manifold constraints with the dynamics
    system.SetForcePremodifier([&](const VectorN& force, const gtfo::DynamicsBase<2>& system){
       const VectorN modified_force = manifold_constraints.Step(force, system.GetPosition(), system.GetVelocity());
       std::cout << modified_force.transpose() << ", ";
       return modified_force;
    });

    // Step the system
    system.SetPositionAndVelocity(VectorN(0.1, 1.0), VectorN::Zero());
    std::cout << "Force, Position, Velocity" << std::endl;
    for(unsigned i = 0; i < 500; ++i){
        system.Step(VectorN(-1.0, 0.0));
        std::cout << system.GetPosition().norm() << ", " << system.GetVelocity().transpose() << std::endl;
    }

    // Verify the state is in the constraint manifold to some mild tolerance 
    EXPECT_LT(system.GetPosition().norm(), radius + 0.1);
}