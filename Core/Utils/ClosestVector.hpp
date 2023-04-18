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
        :   work_(nullptr),
            settings_((OSQPSettings*)c_malloc(sizeof(OSQPSettings))),
            data_((OSQPData*)c_malloc(sizeof(OSQPData)))
    {
        data_->n = Dimension;
        data_->m = NumConstraints;

        // Set the objective matrix as the identity
        const c_int P_nnz = Dimension;
        Eigen::Matrix<c_float, P_nnz, 1> P_x = Eigen::Matrix<c_float, P_nnz, 1>::Constant(1.0);
        Eigen::Matrix<c_int, P_nnz, 1> P_i = Eigen::Matrix<c_int, P_nnz, 1>::LinSpaced(0, P_nnz - 1);
        Eigen::Matrix<c_int, P_nnz + 1, 1> P_p = Eigen::Matrix<c_int, P_nnz + 1, 1>::LinSpaced(0, P_nnz);
        data_->P = csc_matrix(data_->n, data_->n, P_nnz, P_x.data(), P_i.data(), P_p.data());

        // Set the objective vector
        Eigen::Matrix<c_float, Dimension, 1> q = Eigen::Matrix<c_float, Dimension, 1>::Zero();
        data_->q = q.data();

        // Set the constraint matrix
        const c_int A_nnz = NumConstraints * Dimension;
        Eigen::Matrix<c_float, A_nnz, 1> A_x = Eigen::Matrix<c_float, A_nnz, 1>::Zero();
        Eigen::Matrix<c_int, A_nnz, 1> A_i = Eigen::Matrix<c_int, NumConstraints, 1>::LinSpaced(0, NumConstraints - 1).colwise().replicate(Dimension);
        Eigen::Matrix<c_int, Dimension + 1, 1> A_p = Eigen::Matrix<c_int, Dimension + 1, 1>::LinSpaced(0, A_nnz);
        data_->A = csc_matrix(data_->m, data_->n, A_nnz, A_x.data(), A_i.data(), A_p.data());
        
        // Set the lower and upper constraint vectors
        Eigen::Matrix<c_float, NumConstraints, 1> l = Eigen::Matrix<c_float, NumConstraints, 1>::Constant(-OSQP_INFTY);
        data_->l = l.data();
        Eigen::Matrix<c_float, NumConstraints, 1> u = Eigen::Matrix<c_float, NumConstraints, 1>::Zero();
        data_->u = u.data();

        // Solver settings
        osqp_set_default_settings(settings_);

        // Setup workspace
        osqp_setup(&work_, data_, settings_);
    }

    ~ClosestVector(){
        osqp_cleanup(work_);
        if(data_){
            if(data_->A){
                c_free(data_->A);
            }
            if(data_->P){
                c_free(data_->P);
            }
            c_free(data_);
        }
        if(settings_){
            c_free(settings_);
        }
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
    OSQPWorkspace* work_;
    OSQPSettings* settings_;
    OSQPData* data_;
};

}   // namespace gtfo
