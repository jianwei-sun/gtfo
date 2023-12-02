//----------------------------------------------------------------------------------------------------
// File: ManifoldConstraints.hpp
// Desc: Constrains state to user specified manifold by altering the inputs.
//       Note that a vector relative degree of 2 is required
//----------------------------------------------------------------------------------------------------

#pragma once

// Standard libraries includes
#include <functional>

// Third-party dependencies
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/QR>

// Project-specific
#include "../Utils/Constants.hpp"
#include "../Utils/UtilityFunctions.hpp"

namespace gtfo{
template<unsigned int StateDimension, unsigned int ConstraintDimension, typename Scalar = double>
class ManifoldConstraints{
public:
    static_assert(StateDimension > 0, "State Dimension must be at least 1.");
    static_assert(ConstraintDimension > 0, "Constraint Dimension must be at least 1.");
    static_assert(ConstraintDimension <= StateDimension, "Constraint dimension must be less than or equal to state dimension");
    static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");

    // Vector types
    using VectorN = Eigen::Matrix<Scalar, StateDimension, 1>;
    using VectorK = Eigen::Matrix<Scalar, ConstraintDimension, 1>;
    using Vector2K = Eigen::Matrix<Scalar, 2*ConstraintDimension, 1>;
    using MatrixKN = Eigen::Matrix<Scalar, ConstraintDimension, StateDimension>;
    using MatrixNN = Eigen::Matrix<Scalar, StateDimension, StateDimension>;
    using MatrixNK = Eigen::Matrix<Scalar, StateDimension, ConstraintDimension>;

    // Second-order dynamics
    using StateFunctionBottomHalf = std::function<VectorN(const VectorN&, const VectorN&)>;
    using InputFunctionBottomHalf = std::function<MatrixNN(const VectorN&, const VectorN&)>;

    // Constraint function
    using ConstraintFunction = std::function<VectorK(const VectorN&)>;
    using ConstraintFunctionGradient = std::function<MatrixKN(const VectorN&)>;
    using ConstraintFunctionHessianSlice = std::function<MatrixNN(const VectorN&)>;

    // Tranversal control
    using TransversalGain = Eigen::Matrix<Scalar, ConstraintDimension, 2*ConstraintDimension>;

    ManifoldConstraints()
    :   f_bottom_half_(nullptr),
        g_bottom_half_(nullptr),
        constraint_function_(nullptr),
        constraint_function_gradient_(nullptr),
        constraint_function_hessian_slices_{},
        transversal_gain_(TransversalGain::Zero()),
        gamma_(1.0)
    {}

    // Calculate on each iteration after constraint function and gains have been set to create the new forces
    VectorN Step(const VectorN& force, const VectorN& position, const VectorN& velocity){
        // When callbacks are unset, Step just passes through forces
        if (!constraint_function_ || !f_bottom_half_){
            return force;
        }

        // Evaluate the derivatives of h at the current state
        const MatrixKN constraint_function_gradient = constraint_function_gradient_(position);
        
        // Construct the Lie derivative: LgLf
        const MatrixKN decoupling_matrix_unconditioned = constraint_function_gradient * g_bottom_half_(position, velocity);

        // Find and keep the rows that are numerically nonzero
        const Eigen::Matrix<bool, ConstraintDimension, 1> row_is_nonzero = decoupling_matrix_unconditioned.rowwise().squaredNorm().array() >= GTFO_EQUALITY_COMPARISON_TOLERANCE * std::sqrt(StateDimension);
        const MatrixKN decoupling_matrix = row_is_nonzero.replicate(1, StateDimension).select(decoupling_matrix_unconditioned, 0.0);

        // Construct the Lie derivative: Lf^2
        VectorK affine_term = constraint_function_gradient * f_bottom_half_(position, velocity);
        for (unsigned int i = 0; i < ConstraintDimension; ++i){
            affine_term[i] += (velocity.transpose() * constraint_function_hessian_slices_[i](position) * velocity).value();
        }

        // Form the transversal state and compute its stabilizing control. 
        // Note transversal state is in the form [h_i; Lfh_i], so we need to interleave the constraint function and its Lie derivative
        const Eigen::Matrix<Scalar, 2, ConstraintDimension> transversal_state = (Eigen::Matrix<Scalar, 2, ConstraintDimension>() << constraint_function_(position).transpose(), (constraint_function_gradient * velocity).transpose()).finished();
        const VectorK transversal_control = -transversal_gain_ * Vector2K::Map(transversal_state.data());

        // Some rows of the decoupling matrix are zero, but may not appear so due to precision. 
        const MatrixNK pinv_decoupling_matrix = decoupling_matrix.completeOrthogonalDecomposition().pseudoInverse();

        // Assemble the total force by replacing components that align with the nonzero rows of the decoupling matrix, with the transversal control
        VectorN modified_force = force;
        for(unsigned int i = 0; i < ConstraintDimension; ++i){
            if(row_is_nonzero[i]){
                modified_force += pinv_decoupling_matrix.col(i) * (-gamma_ * decoupling_matrix.row(i) * force + sgn(gamma_) * (transversal_control[i] - affine_term[i]));
            }
        }

        return modified_force;
    }

    // Require the user to set the second order dynamics. Since these manifold constraints are only for systems with vector relative degree 2, only the bottom half of f and g matter
    void SetSecondOrderDynamics(const StateFunctionBottomHalf& f_bottom_half, const InputFunctionBottomHalf& g_bottom_half){
        f_bottom_half_ = f_bottom_half;
        g_bottom_half_ = g_bottom_half;
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
    template<bool ConstraintDimensionGreaterThanOne = (ConstraintDimension > 1)>
    void SetTransversalGain(const std::enable_if_t<ConstraintDimensionGreaterThanOne, Eigen::Matrix<Scalar, 1, 2>>& transversal_gain_i){
        assert((transversal_gain_i.array() >= 0).all());
        transversal_gain_ = Eigen::kroneckerProduct(Eigen::Matrix<Scalar, ConstraintDimension, ConstraintDimension>::Identity(), transversal_gain_i);
    }

    // Getters and setters for constraint strength
    [[nodiscard]] Scalar GetConstraintStrength() const {
        return gamma_;
    }

    void SetConstraintStrength(const Scalar& gamma) {
        if(isnan(gamma)){
            return; 
        } else{
            // Constraint to [0,1]
            gamma_ = (gamma > 1.0) ? 1.0 : (gamma < 0.0) ? 0.0 : gamma;
        }
    }

private:
    // Second-order dynamics
    StateFunctionBottomHalf f_bottom_half_;
    InputFunctionBottomHalf g_bottom_half_;

    // Constraint function
    ConstraintFunction constraint_function_;
    ConstraintFunctionGradient constraint_function_gradient_;
    std::array<ConstraintFunctionHessianSlice, ConstraintDimension> constraint_function_hessian_slices_;

    // Tranversal control
    TransversalGain transversal_gain_;

    // Constraint strength
    Scalar gamma_;
};

} // namespace gtfo
