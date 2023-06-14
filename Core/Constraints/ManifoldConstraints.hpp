//----------------------------------------------------------------------------------------------------
// File: ManifoldConstraints.hpp
// Desc: Constrains state to user specified manifold by altering the inputs
//----------------------------------------------------------------------------------------------------

#pragma once

// Standard libraries includes
#include <functional>

// Third-party dependencies
#include <Eigen/Dense>

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

    ManifoldConstraints()
    :   constraint_function_(nullptr),
        constraint_function_gradient_(nullptr),
        constraint_function_hessian_slices_{}
    {}

    VectorN Step(const VectorN& force, const VectorN& position, const VectorN& velocity){
        if (!constraint_function_){
            return force;
        }
    }

    void SetConstraintFunction(const ConstraintFunction& constraint_function, const ConstraintFunctionGradient& constraint_function_gradient, const std::array<ConstraintFunctionHessianSlice, ConstraintDimension>& constraint_function_hessian_slices){
        constraint_function_ = constraint_function;
        constraint_function_gradient_ = constraint_function_gradient;
        constraint_function_hessian_slices_ = constraint_function_hessian_slices;
    }

private:
    ConstraintFunction constraint_function_;
    ConstraintFunctionGradient constraint_function_gradient_;
    std::array<ConstraintFunctionHessianSlice, ConstraintDimension> constraint_function_hessian_slices_;
};

} // namespace gtfo
