//----------------------------------------------------------------------------------------------------
// File: ClosestVector.hpp
// Desc: Utility class for finding the closest vector to a target vector given linear constraints
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <array>
#include <algorithm>
#include <numeric>

// Third-party dependencies
#include <Eigen/Dense>
#include <osqp.h>

namespace gtfo {

template<unsigned Dimension, unsigned NumConstraints>
class ClosestVector{
public:
    using VectorN = Eigen::Matrix<c_float, Dimension, 1>;

    ClosestVector()
        :   data_((OSQPData*)c_malloc(sizeof(OSQPData))),
            settings_((OSQPSettings*)c_malloc(sizeof(OSQPSettings))),
            work_(nullptr)
    {
        data_->n = Dimension;
        data_->m = NumConstraints;

        // Set the objective matrix as the identity
        const c_int P_nnz = Dimension;
        data_->P = csc_spalloc(data_->n, data_->n, P_nnz, 1, 0);
        Eigen::Matrix<c_float, P_nnz, 1>::Map(data_->P->x) = Eigen::Matrix<c_float, P_nnz, 1>::Constant(1.0);
        Eigen::Matrix<c_int, P_nnz, 1>::Map(data_->P->i) = Eigen::Matrix<c_int, P_nnz, 1>::LinSpaced(0, P_nnz - 1);
        Eigen::Matrix<c_int, Dimension + 1, 1>::Map(data_->P->p) = Eigen::Matrix<c_int, Dimension + 1, 1>::LinSpaced(0, Dimension);


        // Set the objective vector
        data_->q = (c_float*)c_malloc(Dimension * sizeof(c_float));
        Eigen::Matrix<c_float, Dimension, 1>::Map(data_->q).setZero();

        // Set the constraint matrix
        const c_int A_nnz = NumConstraints * Dimension;
        data_->A = csc_spalloc(data_->m, data_->n, A_nnz, 1, 0);
        Eigen::Matrix<c_float, A_nnz, 1>::Map(data_->A->x).setZero();
        Eigen::Matrix<c_int, A_nnz, 1>::Map(data_->A->i) = Eigen::Matrix<c_int, NumConstraints, 1>::LinSpaced(0, NumConstraints - 1).colwise().replicate(Dimension);
        Eigen::Matrix<c_int, Dimension + 1, 1>::Map(data_->A->p) = Eigen::Matrix<c_int, Dimension + 1, 1>::LinSpaced(0, A_nnz);
        
        // Set the lower and upper constraint vectors
        data_->l = (c_float*)c_malloc(NumConstraints * sizeof(c_float));
        Eigen::Matrix<c_float, NumConstraints, 1>::Map(data_->l) = Eigen::Matrix<c_float, NumConstraints, 1>::Constant(-OSQP_INFTY);
        data_->u = (c_float*)c_malloc(NumConstraints * sizeof(c_float));
        Eigen::Matrix<c_float, NumConstraints, 1>::Map(data_->u).setZero();

        // Solver settings
        osqp_set_default_settings(settings_);

        // Setup workspace
        osqp_setup(&work_, data_, settings_);
    }

    ~ClosestVector(){
        osqp_cleanup(work_);
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
    ClosestVector(const ClosestVector& other){
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
        osqp_setup(&work_, data_, settings_);
    }

    // Swap function for implementing the copy-and-swap idiom
    friend void swap(ClosestVector& first, ClosestVector& second) noexcept{
        using std::swap;
        swap(first.data_, second.data_);
        swap(first.settings_, second.settings_);
        swap(first.work_, second.work_);
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
        osqp_update_A(work_, constraint_matrix.data(), OSQP_NULL, NumConstraints * Dimension);
    }

    // If the constraint matrix is not updated before this function is called, then the problem is just an unconstrained QP
    VectorN SolveForVector(const VectorN& desired){
        // The cost vector is the negative of the desired vector
        const VectorN cost_vector = -desired;
        osqp_update_lin_cost(work_, cost_vector.data());

        // It seems that status is always 0, which doesn't match one of the expected return types,
        // so just return the solution
        const c_int status = osqp_solve(work_);
        return VectorN::Map(work_->solution->x);
    }
private:
    OSQPData* data_;
    OSQPSettings* settings_;
    OSQPWorkspace* work_;
};

}   // namespace gtfo
