//----------------------------------------------------------------------------------------------------
// File: Manipulator.hpp
// Desc: class representing a physical manipulator
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <type_traits>
#include <vector>
#include <memory>

// Third-party dependencies
#include <Eigen/Dense>
#include <osqp++.h>

// Project-specific
#include "Entity.hpp"
#include "../Models/DynamicsBase.hpp"

namespace gtfo{
namespace collision{

template<unsigned int JointSpaceDimesion, typename Scalar = double>
class Manipulator : public Entity<Scalar>{
public:
    static_assert(JointSpaceDimesion >= 1, "JointSpaceDimension must be at least 1");

    using Vector3 = typename Entity<Scalar>::Vector3;
    // using VectorN = typename DynamicsBase<VirtualDimension, Scalar>::VectorN;
    using JointVector = Eigen::Matrix<Scalar, JointSpaceDimesion, 1>;

    Manipulator(const std::vector<Vector3>& vertices)
        :   Entity<Scalar>(vertices, false),
            number_of_vertices_(vertices.size())//,
            // virtual_model_(nullptr)
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
        
        // For now there are no active collisions nor joint limits
        const Scalar kInfinity = std::numeric_limits<Scalar>::infinity();
        qp_instance_.lower_bounds = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(JointSpaceDimesion, 1, -kInfinity);
        qp_instance_.upper_bounds = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(JointSpaceDimesion, 1, kInfinity);

        // Initialize the solver
        const auto status = qp_solver_.Init(qp_instance_, qp_settings_);
        assert(status.ok());
    }

    void SetJointLimits(const JointVector& lower_bound, const JointVector& upper_bound){
        assert((upper_bound.array() >= lower_bound.array()).all());
        qp_instance_.lower_bounds.head<JointSpaceDimesion>() = lower_bound;
        qp_instance_.upper_bounds.head<JointSpaceDimesion>() = upper_bound;
        const auto status = qp_solver_.SetBounds(qp_instance_.lower_bounds, qp_instance_.upper_bounds);
        assert(status.ok());
    }

    void UpdateVertices(const std::vector<Vector3>& vertices) override{
        assert(vertices.size() == number_of_vertices_);
        Entity<Scalar>::vertices_ = vertices;
        Entity<Scalar>::UpdateSegments();
    }

    // template<typename T>
    // void SetVirtualModel(const T& virtual_model){
    //     static_assert(std::is_base_of_v<DynamicsBase<VirtualDimension, Scalar>, T>, "Virtual model must inherit from DynamicsBase");
    //     virtual_model_ = std::make_shared<T>(virtual_model);
    // }

    // bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)){
    //     // get desired virtual state
    //     // transform to joint space command
    //     // run qp with current collision vectors
    //     // update virtual state with optimal joint space vector

    //     // maybe this function is not even needed
    // }

    void ComputeCollisions(const Entity<Scalar>& other, const Scalar& tol) override{
        Entity<Scalar>::ComputeCollisions(other, tol);
        // update qp matrices
    }

    JointVector GetSafeJointSpaceVelocity(const JointVector& desired_velocity) const{
        
        qp_solver_.SetObjectiveVector(-desired_velocity);

        // run qp
        const osqp::OsqpExitCode exit_code = qp_solver_.Solve();
        // assume exit_code == OsqpExitCode::kOptimal
        return Eigen::Ref<JointVector>(qp_solver_.primal_solution());
    }

private:
    const size_t number_of_vertices_;
    // std::shared_ptr<DynamicsBase<VirtualDimension, Scalar>> virtual_model_;

    osqp::OsqpInstance qp_instance_;
    osqp::OsqpSolver qp_solver_;
    osqp::OsqpSettings qp_settings_;
};

}   // namespace collision
}   // namespace gtfo