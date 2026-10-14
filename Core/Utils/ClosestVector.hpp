//----------------------------------------------------------------------------------------------------
// File: ClosestVector.hpp
// Desc: Utility class for finding the closest vector to a target vector given linear constraints
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <array>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

// Third-party dependencies
#include <Eigen/Dense>
#include <osqp.h>
#include <type_traits>

namespace gtfo {

template<unsigned Dimension, unsigned NumConstraints, typename Scalar = double>
class ClosestVector{
public:
    static_assert(std::is_same_v<c_float, Scalar>, "OSQP's c_float must be of same type as template argument Scalar");

    using VectorN = Eigen::Matrix<c_float, Dimension, 1>;
    using ConstraintVector = Eigen::Matrix<c_float, NumConstraints, 1>;

    ClosestVector()
        :   data_((OSQPData*)c_malloc(sizeof(OSQPData))),
            settings_((OSQPSettings*)c_malloc(sizeof(OSQPSettings))),
            work_(nullptr),
            lower_bound_(Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Constant(-OSQP_INFTY)),
            upper_bound_((Eigen::Matrix<c_float, NumConstraints + Dimension, 1>() <<
                Eigen::Matrix<c_float, NumConstraints, 1>::Zero(), Eigen::Matrix<c_float, Dimension, 1>::Constant(OSQP_INFTY)
                ).finished())
    {
        data_->n = Dimension;
        data_->m = NumConstraints + Dimension;

        // Set the objective matrix as the identity
        const c_int P_nnz = Dimension;
        data_->P = csc_spalloc(data_->n, data_->n, P_nnz, 1, 0);
        Eigen::Matrix<c_float, P_nnz, 1>::Map(data_->P->x) = Eigen::Matrix<c_float, P_nnz, 1>::Constant(1.0);
        Eigen::Matrix<c_int, P_nnz, 1>::Map(data_->P->i) = Eigen::Matrix<c_int, P_nnz, 1>::LinSpaced(0, P_nnz - 1);
        Eigen::Matrix<c_int, Dimension + 1, 1>::Map(data_->P->p) = Eigen::Matrix<c_int, Dimension + 1, 1>::LinSpaced(0, Dimension);

        // Set the objective vector
        data_->q = (c_float*)c_malloc(Dimension * sizeof(c_float));
        Eigen::Matrix<c_float, Dimension, 1>::Map(data_->q).setZero();

        // Precompute the indices corresponding to the non-identity portion of the constraint matrix
        for(unsigned i = 0; i < Dimension; ++i){
            for(unsigned j = 0; j < NumConstraints; ++j){
                non_identity_constraint_indices_[i * NumConstraints + j] = i * (NumConstraints + 1) + j;
            }
        }

        // Set the constraint matrix
        const c_int A_nnz = NumConstraints * Dimension + Dimension;
        data_->A = csc_spalloc(data_->m, data_->n, A_nnz, 1, 0);
        Eigen::Matrix<c_float, A_nnz, 1>::Map(data_->A->x).setZero();
        for(unsigned i = NumConstraints; i < A_nnz; i += NumConstraints + 1){
            data_->A->x[i] = (c_float)1.0f;
        }
        Eigen::Matrix<c_int, A_nnz, 1>::Map(data_->A->i) = Eigen::Matrix<c_int, NumConstraints + 1, 1>::LinSpaced(0, NumConstraints).colwise().replicate(Dimension);
        for(unsigned j = 0; j < Dimension; ++j){
            data_->A->i[(NumConstraints + 1) * j + NumConstraints] += static_cast<c_int>(j);
        }
        Eigen::Matrix<c_int, Dimension + 1, 1>::Map(data_->A->p) = Eigen::Matrix<c_int, Dimension + 1, 1>::LinSpaced(0, A_nnz);
        
        // Set the lower and upper constraint vectors
        data_->l = (c_float*)c_malloc((NumConstraints + Dimension) * sizeof(c_float));
        Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Map(data_->l) = lower_bound_;
        data_->u = (c_float*)c_malloc((NumConstraints + Dimension) * sizeof(c_float));
        Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Map(data_->u) = upper_bound_;

        // Solver settings
        osqp_set_default_settings(settings_);

        // Setup workspace
        setup_error_ = osqp_setup(&work_, data_, settings_);
    }

    ~ClosestVector(){
        if(work_) osqp_cleanup(work_);
        if(data_){
            csc_spfree(data_->P);
            csc_spfree(data_->A);
            if(data_->q) c_free(data_->q);
            if(data_->l) c_free(data_->l);
            if(data_->u) c_free(data_->u);
            c_free(data_);
        }
        if(settings_){
            c_free(settings_);
        }
    }

    // Copy constructor
    ClosestVector(const ClosestVector& other) : work_(nullptr){
        // Copy the OSQPData struct
        data_ = (OSQPData*)c_malloc(sizeof(OSQPData));
        data_->n = other.data_->n;
        data_->m = other.data_->m;
        data_->P = copy_csc_mat(other.data_->P);
        data_->A = copy_csc_mat(other.data_->A); 
        data_->q = vec_copy(other.data_->q, other.data_->n);
        data_->l = vec_copy(other.data_->l, other.data_->m);
        data_->u = vec_copy(other.data_->u, other.data_->m);

        settings_ = copy_settings(other.settings_);

        // The workspace is not copied over due to its complexity, but rather created as a new object
        setup_error_ = osqp_setup(&work_, data_, settings_);
        matrix_error_ = other.matrix_error_;
        bounds_error_ = other.bounds_error_;

        // The other variables are copied over too
        lower_bound_ = other.lower_bound_;
        upper_bound_ = other.upper_bound_;
        non_identity_constraint_indices_ = other.non_identity_constraint_indices_;
    }

    // Swap function for implementing the copy-and-swap idiom
    friend void swap(ClosestVector& first, ClosestVector& second) noexcept{
        using std::swap;
        swap(first.data_, second.data_);
        swap(first.settings_, second.settings_);
        swap(first.work_, second.work_);
        swap(first.lower_bound_, second.lower_bound_);
        swap(first.upper_bound_, second.upper_bound_);
        swap(first.non_identity_constraint_indices_, second.non_identity_constraint_indices_);
        swap(first.setup_error_, second.setup_error_);
        swap(first.matrix_error_, second.matrix_error_);
        swap(first.bounds_error_, second.bounds_error_);
        swap(first.solve_error_, second.solve_error_);
        swap(first.solve_status_, second.solve_status_);
        swap(first.last_solve_succeeded_, second.last_solve_succeeded_);
    }

    // Move constructor
    ClosestVector(ClosestVector&& other) noexcept
        :   ClosestVector()
    {
        swap(*this, other);
    }

    // Assignment operator
    ClosestVector& operator=(ClosestVector other){
        swap(*this, other);
        return *this;
    }

    // Eigen matrices are column-major by default. The argument type automatically converts row-major formats into
    // constraint_matrix before its values are used to update A
    void UpdateConstraintMatrix(const Eigen::Matrix<c_float, NumConstraints, Dimension>& constraint_matrix){
        last_solve_succeeded_ = false;
        matrix_error_ = work_ && !setup_error_ && constraint_matrix.allFinite()
            ? osqp_update_A(work_, constraint_matrix.data(), non_identity_constraint_indices_.data(), NumConstraints * Dimension)
            : -1;
        if(!matrix_error_){
            // Keep the unscaled input for copy construction. work_->data->A is scaled.
            for(unsigned i = 0; i < NumConstraints * Dimension; ++i)
                data_->A->x[non_identity_constraint_indices_[i]] = constraint_matrix.data()[i];
        }
    }

    void UpdateConstraintBounds(
        const ConstraintVector& lower_bound,
        const ConstraintVector& upper_bound)
    {
        lower_bound_.template head<NumConstraints>() = lower_bound;
        upper_bound_.template head<NumConstraints>() = upper_bound;

        UpdateBounds();
        Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Map(data_->l) = lower_bound_;
        Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Map(data_->u) = upper_bound_;
    }

    void SetDimensionBounds(
        const size_t& dimension,
        const Scalar& lower_bound,
        const Scalar& upper_bound)
    {
        lower_bound_[NumConstraints + dimension] = lower_bound;
        upper_bound_[NumConstraints + dimension] = upper_bound;

        UpdateBounds();
        Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Map(data_->l) = lower_bound_;
        Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Map(data_->u) = upper_bound_;
    }
    
    void SetDimensionFixed(const size_t& dimension, const bool& fixed){
        if(fixed){
            lower_bound_[NumConstraints + dimension] = 0.0f;
            upper_bound_[NumConstraints + dimension] = 0.0f;
        } else{
            lower_bound_[NumConstraints + dimension] = -OSQP_INFTY;
            upper_bound_[NumConstraints + dimension] = OSQP_INFTY;
        }
        UpdateBounds();
        Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Map(data_->l) = lower_bound_;
        Eigen::Matrix<c_float, NumConstraints + Dimension, 1>::Map(data_->u) = upper_bound_;
    }

    // If the constraint matrix is not updated before this function is called, then the problem is just an unconstrained QP
    VectorN SolveForVector(const VectorN& desired){
        last_solve_succeeded_ = false;
        solve_status_ = OSQP_UNSOLVED;
        solve_error_ = 0;
        if(!work_ || setup_error_ || matrix_error_ || bounds_error_ || !desired.allFinite()){
            solve_error_ = -1;
            return VectorN::Constant(std::numeric_limits<Scalar>::quiet_NaN());
        }
        // The cost vector is the negative of the desired vector
        Eigen::Matrix<c_float, Dimension, 1>::Map(data_->q) = -desired;
        solve_error_ = osqp_update_lin_cost(work_, data_->q);
        if(solve_error_)
            return VectorN::Constant(std::numeric_limits<Scalar>::quiet_NaN());

        // API return code and optimization status are distinct in OSQP 0.6.
        solve_error_ = osqp_solve(work_);
        solve_status_ = work_->info->status_val;
        last_solve_succeeded_ = !solve_error_ && solve_status_ == OSQP_SOLVED
            && VectorN::Map(work_->solution->x).allFinite();
        // Preserve the legacy return type and solver output; checked users inspect status.
        return VectorN::Map(work_->solution->x);
    }

    bool LastSolveSucceeded() const{ return last_solve_succeeded_; }
    c_int GetSolveStatus() const{ return solve_status_; }
    c_int GetSetupError() const{ return setup_error_; }
    c_int GetMatrixUpdateError() const{ return matrix_error_; }
    c_int GetBoundsUpdateError() const{ return bounds_error_; }
    c_int GetSolveError() const{ return solve_error_; }

    // Opt-in only: construction still uses OSQP's original defaults.
    bool SetSolverTolerances(const Scalar& absolute, const Scalar& relative, c_int max_iterations){
        if(!work_ || setup_error_ || !std::isfinite(absolute) || !std::isfinite(relative)
            || absolute <= 0 || relative <= 0 || max_iterations <= 0) return false;
        last_solve_succeeded_ = false;
        if(osqp_update_eps_abs(work_, absolute) || osqp_update_eps_rel(work_, relative)
            || osqp_update_max_iter(work_, max_iterations)) return false;
        settings_->eps_abs = absolute;
        settings_->eps_rel = relative;
        settings_->max_iter = max_iterations;
        return true;
    }
private:
    void UpdateBounds(){
        last_solve_succeeded_ = false;
        bounds_error_ = work_ && !setup_error_ && !lower_bound_.array().isNaN().any()
            && !upper_bound_.array().isNaN().any()
            ? osqp_update_bounds(work_, lower_bound_.data(), upper_bound_.data()) : -1;
    }

    c_int setup_error_ = 0;
    c_int matrix_error_ = 0;
    c_int bounds_error_ = 0;
    c_int solve_error_ = 0;
    c_int solve_status_ = OSQP_UNSOLVED;
    bool last_solve_succeeded_ = false;
    OSQPData* data_;
    OSQPSettings* settings_;
    OSQPWorkspace* work_;

    Eigen::Matrix<c_float, NumConstraints + Dimension, 1> lower_bound_;
    Eigen::Matrix<c_float, NumConstraints + Dimension, 1> upper_bound_;
    std::array<c_int, NumConstraints * Dimension> non_identity_constraint_indices_;
};

}   // namespace gtfo
