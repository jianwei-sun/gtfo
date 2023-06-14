//----------------------------------------------------------------------------------------------------
// File: ManifoldConstraints.hpp
// Desc: Constrains state to user specified manifold by altering the inputs
//----------------------------------------------------------------------------------------------------

#pragma once

// Standard libraries includes
#include <functional>

// Third-party dependencies
#include <Eigen/Dense>
#include <Eigen/KroneckerProduct>


// Project-specific


namespace gtfo{
template<unsigned int StateDimension, unsigned int ConstraintDimension, typename Scalar = double>

class ManifoldConstraints{
public:
    static_assert(StateDimension > 0, "State Dimension must be at least 1.");
    static_assert(ConstraintDimension > 0, "Constraint Dimension must be at least 1.");
    static_assert(ConstraintDimension <= StateDimension, "Constraint dimension must be less than or equal to state dimension");
    static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");

    using VectorN = Eigen::Matrix<Scalar, StateDimension, 1>;
    using VectorK = Eigen::Matrix<Scalar, ConstraintDimension, 1>;
    using MatrixKN = Eigen::Matrix<Scalar, ConstraintDimension, StateDimension>;
    using MatrixNN = Eigen::Matrix<Scalar, StateDimension, StateDimension>;

    using ConstraintFunction = std::function<VectorK(const VectorN&)>;
    using ConstraintFunctionGradient = std::function<MatrixKN(const VectorN&)>;
    using ConstraintFunctionHessianSlice = std::function<MatrixNN(const VectorN&)>;
    using TransversalGain = Eigen::Matrix<Scalar, ConstraintDimension, 2*ConstraintDimension>;

    ManifoldConstraints()
    :   constraint_function_(nullptr),
        constraint_function_gradient_(nullptr),
        constraint_function_hessian_slices_{},
        transversal_gain_(TransversalGain::Zero())
    {}

    // Calculate on each iteration after constraint function and gains have been set to create the new forces
    VectorN Step(const VectorN& force, const VectorN& position, const VectorN& velocity){
        if (!constraint_function_){
            return force;
        }

    }

    // Set constraint function, and first and second partial derivative. Because second partial is a tensor, we enter hessian slices instead
    void SetConstraintFunction(const ConstraintFunction& constraint_function, const ConstraintFunctionGradient& constraint_function_gradient, const std::array<ConstraintFunctionHessianSlice, ConstraintDimension>& constraint_function_hessian_slices){
        constraint_function_ = constraint_function;
        constraint_function_gradient_ = constraint_function_gradient;
        constraint_function_hessian_slices_ = constraint_function_hessian_slices;
    }

    // Transversal gain in the general case where the whole matrix is entered. This is for when the gains are different in each constraint coordinate
    void SetTransversalGain(const Eigen::Matrix<Scalar, ConstraintDimension, 2*ConstraintDimension>& transversal_gain){
        assert((transversal_gain.array() >= 0).all());

        transversal_gain_ = transversal_gain;
    }

    // Transversal Gain when the gains are similar for all constraint coordinates
    void SetTransversalGain(const Eigen::Matrix<Scalar, 1, 2>& transversal_gain_i){
        assert((transversal_gain_i.array() >= 0).all());

        transversal_gain_ = Eigen::kroneckerProduct (Eigen::Matrix<Scalar, ConstraintDimension, ConstraintDimension>::Identity(), transversal_gain_i);
    }

private:
    ConstraintFunction constraint_function_;
    ConstraintFunctionGradient constraint_function_gradient_;
    std::array<ConstraintFunctionHessianSlice, ConstraintDimension> constraint_function_hessian_slices_;
    TransversalGain transversal_gain_;
};

} // namespace gtfo
