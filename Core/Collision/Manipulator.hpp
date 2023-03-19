//----------------------------------------------------------------------------------------------------
// File: Manipulator.hpp
// Desc: class representing a physical manipulator
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <type_traits>
#include <vector>
#include <memory>
#include <functional>

// Third-party dependencies
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp++.h>

// Project-specific
#include "Eigen/src/Core/util/Constants.h"
#include "Entity.hpp"

namespace gtfo{
namespace collision{

template<unsigned int JointSpaceDimesion, typename Scalar = double>
class Manipulator : public Entity<Scalar>{
public:
    static_assert(JointSpaceDimesion >= 1, "JointSpaceDimension must be at least 1");

    using Vector3 = typename Entity<Scalar>::Vector3;
    using JointVector = Eigen::Matrix<Scalar, JointSpaceDimesion, 1>;

    Manipulator(const std::vector<Vector3>& vertices)
        :   Entity<Scalar>(vertices, false),
            number_of_vertices_(vertices.size()),
            partial_jacobian_getter_(nullptr)
    {
        // Sets the objective matrix
        Eigen::SparseMatrix<Scalar> objective_matrix(JointSpaceDimesion, JointSpaceDimesion);
        objective_matrix.setIdentity();
        qp_instance_.objective_matrix = objective_matrix;

        // The objective vector depends on the desired joint velocity
        qp_instance_.objective_vector.resize(JointSpaceDimesion);
        qp_instance_.objective_vector.setZero();

        // The constraint matrix has as many rows as active collisions plus joint limits
        Eigen::SparseMatrix<Scalar> constraint_matrix(JointSpaceDimesion, JointSpaceDimesion);
        constraint_matrix.setIdentity();
        qp_instance_.constraint_matrix = constraint_matrix;
        
        // For now there are no active collisions nor joint velocity limits
        const Scalar kInfinity = std::numeric_limits<Scalar>::infinity();
        qp_instance_.lower_bounds = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(JointSpaceDimesion, 1, -kInfinity);
        qp_instance_.upper_bounds = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(JointSpaceDimesion, 1, kInfinity);

        // Initialize the solver
        qp_solver_ = std::make_shared<osqp::OsqpSolver>();
        const auto status = qp_solver_->Init(qp_instance_, qp_settings_);
        assert(status.ok());
    }

    // Updates the locations of the collision points. Verifies that the number of vertices matches the number
    // of vertices given at construction
    void UpdateVertices(const std::vector<Vector3>& vertices) override{
        assert(vertices.size() == number_of_vertices_);
        Entity<Scalar>::vertices_ = vertices;
        Entity<Scalar>::UpdateSegments();
    }

    // To enable collision avoidance, a partial jacobian (first three rows) which can be evaluated at any arbitrary point
    // is needed
    void EnableCollisionAvoidance(const std::function<Eigen::Matrix<Scalar, 3, Eigen::Dynamic>(const Vector3&)>& partial_jacobian_getter){
        partial_jacobian_getter_ = partial_jacobian_getter;
    }

    // Collision avoidance can also ensure that joint velocities do not exceed certain bounds
    void EnableJointVelocityLimits(const JointVector& lower_bound, const JointVector& upper_bound){
        assert((upper_bound.array() >= lower_bound.array()).all());
        qp_instance_.lower_bounds.head<JointSpaceDimesion>() = lower_bound;
        qp_instance_.upper_bounds.head<JointSpaceDimesion>() = upper_bound;
        const auto status = qp_solver_->SetBounds(qp_instance_.lower_bounds, qp_instance_.upper_bounds);
        assert(status.ok());
    }

    // Returns the closest joint-space velocity to desired_velocity while still respecting joint velocities
    // and avoiding collisions. Note that all calls to ComputeCollisions should be completed before calling
    // this function, if collision avoidance is enabled
    JointVector GetSafeJointSpaceVelocity(const JointVector& desired_velocity){
        // If collision avoidance is enabled, update the constraint matrix and vectors to avoid velocities
        // that move farther into the collision
        if(partial_jacobian_getter_){
            const size_t num_collisions = Entity<Scalar>::collisions_.size();

            // Expand the constraint matrix
            qp_instance_.constraint_matrix.resize(JointSpaceDimesion + num_collisions, JointSpaceDimesion);
            qp_instance_.constraint_matrix.reserve(JointSpaceDimesion + num_collisions * JointSpaceDimesion);

            for(unsigned i = 0; i < num_collisions; ++i){
                // Formulate the collision avoidance as a linear constraint on velocity
                const Segment<Scalar>& collision = Entity<Scalar>::collisions_[i];
                const Eigen::Matrix<Scalar, 3, Eigen::Dynamic> partial_jacobian = partial_jacobian_getter_(collision.Start());
                const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> constraint_row = collision.End().transpose() * partial_jacobian;

                // Copy the row into the constraint matrix
                for(Eigen::Index j{0}; j < constraint_row.cols(); ++j){
                    qp_instance_.constraint_matrix.coeffRef(JointSpaceDimesion + i, j) = constraint_row.coeffRef(j);
                }
            }

            auto status = qp_solver_->UpdateConstraintMatrix(qp_instance_.constraint_matrix);

            // Lower bounds that correspond to collision avoidance are just at negative infinity
            qp_instance_.lower_bounds.resize(JointSpaceDimesion + num_collisions);
            qp_instance_.lower_bounds.tail(num_collisions) = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(num_collisions, 1, -std::numeric_limits<Scalar>::infinity());
            
            // Upper bounds corresponding to collision avoidance are at zero
            qp_instance_.upper_bounds.resize(JointSpaceDimesion + num_collisions);
            qp_instance_.lower_bounds.tail(num_collisions).setZero();

            status = qp_solver_->SetBounds(qp_instance_.lower_bounds, qp_instance_.upper_bounds);
        }

        // Regardess of whether collision avoidance is enabled, the desired_velocity appears in the QP as
        // the negative of the objective vector
        auto status = qp_solver_->SetObjectiveVector(-desired_velocity);

        // Solve the QP and return a non-zero value if the optimal solution is found
        if(qp_solver_->Solve() ==  osqp::OsqpExitCode::kOptimal){
            return JointVector(qp_solver_->primal_solution());
        } else{
            return JointVector::Zero();
        }
    }

private:
    const size_t number_of_vertices_;

    // A callback for computing the first three rows of the Jacobian along any arbitrary point
    std::function<Eigen::Matrix<Scalar, 3, Eigen::Dynamic>(const Vector3&)> partial_jacobian_getter_;

    // QP-solver member variables. Note that qp_instance_ just stores a copy of the instance set in qp_solver_
    osqp::OsqpInstance qp_instance_;
    std::shared_ptr<osqp::OsqpSolver> qp_solver_;
    osqp::OsqpSettings qp_settings_;
};

}   // namespace collision
}   // namespace gtfo