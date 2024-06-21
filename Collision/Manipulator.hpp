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

// Project-specific
#include "Entity.hpp"
#include "../Core/Utils/ClosestVector.hpp"
#include "../Core/Models/DynamicsBase.hpp"

namespace gtfo{
namespace collision{

template<unsigned int JointSpaceDimension, unsigned int MaxCollisionsPerSegment, unsigned int VirtualDimension = JointSpaceDimension, typename Scalar = double>
class Manipulator : public Entity<Scalar>{
public:
    static_assert(JointSpaceDimension >= 1, "JointSpaceDimension must be at least 1");
    static_assert(MaxCollisionsPerSegment >= 1, "MaxCollisionsPerSegment must be at least 1");
    static_assert(VirtualDimension >= 1, "VirtualDimension must be at least 1");

    using Vector3 = typename Entity<Scalar>::Vector3;
    using JointVector = Eigen::Matrix<Scalar, JointSpaceDimension, 1>;
    using PartialJacobian = Eigen::Matrix<Scalar, 3, JointSpaceDimension, Eigen::RowMajor>;
    using VirtualVector = Eigen::Matrix<Scalar, VirtualDimension, 1>;

    Manipulator(const std::vector<Vector3>& vertices)
        :   Entity<Scalar>(vertices, false),
            number_of_vertices_(vertices.size()),
            partial_jacobian_updater_(nullptr),
            partial_jacobian_(PartialJacobian::Zero()),
            model_ptr_(nullptr),
            virtual_to_joint_(nullptr),
            joint_to_virtual_(nullptr)
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

    void SetVirtualDynamics(DynamicsBase<VirtualDimension, Scalar, VirtualDimension + 1> *model_ptr, const std::function<JointVector(const VirtualVector &)> &virtual_to_joint, const std::function<VirtualVector(const JointVector &)> &joint_to_virtual)
    {
        model_ptr_ = model_ptr;
        virtual_to_joint_ = virtual_to_joint;
        joint_to_virtual_ = joint_to_virtual;
    }

    void  UpdateVirtualState(void) override{
        if(!model_ptr_ || !virtual_to_joint_ || !joint_to_virtual_){
            return;
        }
        // const VirtualVector& position = model_ptr_->GetPosition();
        const Eigen::Matrix<Scalar, VirtualDimension + 1, 1>& position = model_ptr_->GetPosition();
        const VirtualVector& velocity = model_ptr_->GetVelocity();
        const VirtualVector constrained_virtual_velocity = joint_to_virtual_(GetSafeJointSpaceVelocity(virtual_to_joint_(velocity)));
        if(!IsEqual(constrained_virtual_velocity, velocity)){
            const VirtualVector normal = (velocity - constrained_virtual_velocity).normalized();
            Eigen::Matrix<Scalar, VirtualDimension + 1, 1> normal_expand;
            normal_expand.setZero();
            normal_expand.block(0, 0, 3, 1) = normal.block(0, 0, 3, 1);
            normal_expand.block(4, 0, 3, 1) = normal.block(3, 0, 3, 1);

            model_ptr_->SetState(
                position - (position - model_ptr_->GetOldPosition()).dot(normal_expand) * normal_expand,
                constrained_virtual_velocity
            );
        }
    }

    // Returns the closest joint-space velocity to desired_velocity while still avoiding collisions. 
    // Note that all calls to ComputeCollisions should be completed before calling this function, 
    // if collision avoidance is enabled
    JointVector GetSafeJointSpaceVelocity(const JointVector& desired_velocity){
        // If collision avoidance is enabled, update the constraint matrix to avoid velocities
        // that move farther into the collision
        if(partial_jacobian_updater_){
            Eigen::Matrix<Scalar, MaxCollisionsPerSegment, JointSpaceDimension> constraint_matrix = Eigen::Matrix<Scalar, MaxCollisionsPerSegment, JointSpaceDimension>::Zero();

            for(unsigned i = 0; i < std::min<size_t>(Entity<Scalar>::collisions_.size(), MaxCollisionsPerSegment); ++i){
                const Collision<Scalar>& collision = Entity<Scalar>::collisions_[i];
                partial_jacobian_updater_(partial_jacobian_, collision.segment_index_, collision.location_);
                constraint_matrix.template block<1, JointSpaceDimension>(i, 0) = collision.direction_.transpose() * partial_jacobian_;
            }

            solver_.UpdateConstraintMatrix(constraint_matrix);
        }

        // If the jacobians are not available, then only solve with velocity constraints
        return solver_.SolveForVector(desired_velocity);
    }

    void SetJointFixed(const size_t& joint, const bool& fixed){
        solver_.SetDimensionFixed(joint, fixed);
    }

private:
    const size_t number_of_vertices_;

    // A callback for computing the first three rows of the Jacobian along any arbitrary point
    std::function<void(PartialJacobian&, const size_t&, const Vector3&)> partial_jacobian_updater_;
    PartialJacobian partial_jacobian_;

    ClosestVector<JointSpaceDimension, MaxCollisionsPerSegment, Scalar> solver_;

    // Virtual dynamics related
    DynamicsBase<VirtualDimension, Scalar, VirtualDimension + 1> *model_ptr_;
    // DynamicsBase<7, Scalar, 4> *model_ptr_;
    std::function<JointVector(const VirtualVector&)> virtual_to_joint_;
    std::function<VirtualVector(const JointVector&)> joint_to_virtual_;
};

}   // namespace collision
}   // namespace gtfo