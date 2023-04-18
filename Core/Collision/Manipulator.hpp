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
// #include <Eigen/Sparse>
// #include <osqp++.h>



// Project-specific
#include "Entity.hpp"
#include "../Utils/ClosestVector.hpp"

namespace gtfo{
namespace collision{

template<unsigned int JointSpaceDimension, unsigned int MaxCollisionsPerSegment, typename Scalar = double>
class Manipulator : public Entity<Scalar>{
public:
    static_assert(JointSpaceDimension >= 1, "JointSpaceDimension must be at least 1");
    static_assert(MaxCollisionsPerSegment >= 1, "MaxCollisionsPerSegment must be at least 1");

    using Vector3 = typename Entity<Scalar>::Vector3;
    using JointVector = Eigen::Matrix<Scalar, JointSpaceDimension, 1>;
    using PartialJacobian = Eigen::Matrix<Scalar, 3, JointSpaceDimension, Eigen::RowMajor>;

    Manipulator(const std::vector<Vector3>& vertices)
        :   Entity<Scalar>(vertices, false),
            number_of_vertices_(vertices.size()),
            partial_jacobian_updater_(nullptr),
            partial_jacobian_(PartialJacobian::Zero()),
            solver_(nullptr)
    {}

    // Updates the locations of the collision points. Verifies that the number of vertices matches the number
    // of vertices given at construction
    void UpdateVertices(const std::vector<Vector3>& vertices) override{
        assert(vertices.size() == number_of_vertices_);
        Entity<Scalar>::vertices_ = vertices;
        Entity<Scalar>::UpdateSegments();
    }

    // To enable collision avoidance, a partial jacobian (first three rows) which can be evaluated at any arbitrary point
    // is needed
    void EnableCollisionAvoidance(const std::function<void(PartialJacobian&, const size_t&, const Vector3&)>& partial_jacobian_updater){
        partial_jacobian_updater_ = partial_jacobian_updater;
    }

    void SetSolver(ClosestVector<JointSpaceDimension, MaxCollisionsPerSegment>* solver){
        solver_ = solver;
    }

    // Returns the closest joint-space velocity to desired_velocity while still avoiding collisions. 
    // Note that all calls to ComputeCollisions should be completed before calling this function, 
    // if collision avoidance is enabled
    JointVector GetSafeJointSpaceVelocity(const JointVector& desired_velocity){
        // If collision avoidance is enabled, update the constraint matrix to avoid velocities
        // that move farther into the collision
        if(partial_jacobian_updater_ && solver_){
            Eigen::Matrix<Scalar, MaxCollisionsPerSegment, JointSpaceDimension> constraint_matrix = Eigen::Matrix<Scalar, MaxCollisionsPerSegment, JointSpaceDimension>::Zero();

            for(unsigned i = 0; i < std::min<size_t>(Entity<Scalar>::collisions_.size(), MaxCollisionsPerSegment); ++i){
                const Collision<Scalar>& collision = Entity<Scalar>::collisions_[i];
                partial_jacobian_updater_(partial_jacobian_, collision.segment_index_, collision.location_);
                constraint_matrix.block<1, JointSpaceDimension>(i, 0) = collision.direction_.transpose() * partial_jacobian_;
            }

            solver_->UpdateConstraintMatrix(constraint_matrix);
            return solver_->SolveForVector(desired_velocity);
        }

        // If collision avoidance is not enabled, simply pass the desired_velocity through
        return desired_velocity;
    }

private:
    const size_t number_of_vertices_;

    // A callback for computing the first three rows of the Jacobian along any arbitrary point
    std::function<void(PartialJacobian&, const size_t&, const Vector3&)> partial_jacobian_updater_;
    PartialJacobian partial_jacobian_;

    ClosestVector<JointSpaceDimension, MaxCollisionsPerSegment>* solver_;
};

}   // namespace collision
}   // namespace gtfo