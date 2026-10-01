//----------------------------------------------------------------------------------------------------
// File: CARG.hpp
// Desc: Capture-admissible reference governor
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <array>
#include <functional>
#include <limits>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Core/Models/DynamicsBase.hpp"
#include "../Core/Utils/ClosestVector.hpp"
#include "../Core/Utils/CaptureDistance.hpp"

namespace gtfo{
namespace constraints{

template<unsigned VirtualDimension, unsigned MaxConstraints, typename Scalar = double>
class CARG{
public:
    static_assert(VirtualDimension >= 1, "VirtualDimension must be at least 1");
    static_assert(MaxConstraints >= 1, "MaxConstraints must be at least 1");

    static constexpr unsigned NumSolverConstraints = 3 * MaxConstraints;

    using VirtualVector = Eigen::Matrix<Scalar, VirtualDimension, 1>;
    using ConstraintMatrix = Eigen::Matrix<Scalar, NumSolverConstraints, VirtualDimension>;
    using ConstraintVector = Eigen::Matrix<Scalar, NumSolverConstraints, 1>;

    struct Constraint{
        Scalar clearance_;
        Scalar velocity_;
        Scalar acceleration_;
        VirtualVector gamma_;
        Scalar beta_;

        Constraint()
            :   clearance_(0),
                velocity_(0),
                acceleration_(0),
                gamma_(VirtualVector::Zero()),
                beta_(0)
        {}
    };

    struct ConstraintStatus{
        Scalar capture_distance_;
        Scalar capture_margin_;
        Scalar predicted_capture_margin_;
        bool active_;

        ConstraintStatus()
            :   capture_distance_(0),
                capture_margin_(0),
                predicted_capture_margin_(0),
                active_(false)
        {}
    };

    using ConstraintUpdater =
        std::function<void(Constraint&, const VirtualVector&, const VirtualVector&, const VirtualVector&)>;

    CARG(const Scalar& dt)
        :   dt_(dt),
            model_ptr_(nullptr),
            old_position_(VirtualVector::Zero()),
            old_velocity_(VirtualVector::Zero()),
            old_acceleration_(VirtualVector::Zero()),
            jerk_(VirtualVector::Zero()),
            number_of_constraints_(0)
    {}

    void SetVirtualDynamics(
        DynamicsBase<VirtualDimension, Scalar>* model_ptr,
        const VirtualVector& initial_acceleration = VirtualVector::Zero())
    {
        model_ptr_ = model_ptr;
        old_position_ = model_ptr_->GetPosition();
        old_velocity_ = model_ptr_->GetVelocity();
        old_acceleration_ = initial_acceleration;
    }

    void AddConstraint(
        const ConstraintUpdater& updater,
        const Scalar& max_acceleration,
        const Scalar& max_jerk,
        const Scalar& activation_tolerance = 0.0)
    {
        assert(number_of_constraints_ < MaxConstraints);

        ConstraintData& data = constraints_[number_of_constraints_++];
        data.updater_ = updater;
        data.max_acceleration_ = max_acceleration;
        data.max_jerk_ = max_jerk;
        data.activation_tolerance_ = activation_tolerance;
    }

    void ClearConstraints(void){
        number_of_constraints_ = 0;
    }

    void SetVirtualJerkLimits(const VirtualVector& max_jerk){
        for(unsigned i = 0; i < VirtualDimension; ++i){
            solver_.SetDimensionBounds(i, -max_jerk(i), max_jerk(i));
        }
    }

    void UpdateVirtualState(void){
        if(!model_ptr_){
            return;
        }

        const VirtualVector nominal_position = model_ptr_->GetPosition();
        const VirtualVector nominal_velocity = model_ptr_->GetVelocity();
        const VirtualVector nominal_acceleration =
            (nominal_velocity - old_velocity_) / dt_;

        ConstraintMatrix constraint_matrix = ConstraintMatrix::Zero();
        ConstraintVector lower_bound =
            ConstraintVector::Constant(-OSQP_INFTY);
        ConstraintVector upper_bound =
            ConstraintVector::Constant(OSQP_INFTY);

        for(unsigned i = 0; i < number_of_constraints_; ++i){
            Constraint old_constraint;
            Constraint nominal_constraint;

            constraints_[i].updater_(
                old_constraint,
                old_position_,
                old_velocity_,
                old_acceleration_);

            constraints_[i].updater_(
                nominal_constraint,
                nominal_position,
                nominal_velocity,
                nominal_acceleration);

            const auto capture =
                ComputeCaptureDistance(
                    old_constraint.velocity_,
                    old_constraint.acceleration_,
                    constraints_[i].max_acceleration_,
                    constraints_[i].max_jerk_);

            const auto predicted_capture =
                ComputeCaptureDistance(
                    nominal_constraint.velocity_,
                    nominal_constraint.acceleration_,
                    constraints_[i].max_acceleration_,
                    constraints_[i].max_jerk_);

            ConstraintStatus& status = status_[i];
            status.capture_distance_ = capture.distance_;
            status.capture_margin_ =
                old_constraint.clearance_ - capture.distance_;
            status.predicted_capture_margin_ =
                nominal_constraint.clearance_ - predicted_capture.distance_;

            status.active_ =
                status.capture_margin_ <= constraints_[i].activation_tolerance_
                || status.predicted_capture_margin_ < 0.0;

            if(!status.active_){
                continue;
            }

            const unsigned row = 3 * i;

            constraint_matrix.row(row) =
                (capture.Da_ * old_constraint.gamma_).transpose();
            upper_bound(row) =
                -old_constraint.velocity_
                - capture.Dv_ * old_constraint.acceleration_
                - capture.Da_ * old_constraint.beta_;

            constraint_matrix.row(row + 1) =
                old_constraint.gamma_.transpose();
            lower_bound(row + 1) =
                -constraints_[i].max_jerk_ - old_constraint.beta_;
            upper_bound(row + 1) =
                constraints_[i].max_jerk_ - old_constraint.beta_;

            constraint_matrix.row(row + 2) =
                old_constraint.gamma_.transpose();
            lower_bound(row + 2) =
                (-constraints_[i].max_acceleration_ - old_constraint.acceleration_) / dt_
                - old_constraint.beta_;
            upper_bound(row + 2) =
                (constraints_[i].max_acceleration_ - old_constraint.acceleration_) / dt_
                - old_constraint.beta_;
        }

        const VirtualVector desired_jerk =
            (nominal_acceleration - old_acceleration_) / dt_;

        solver_.UpdateConstraintMatrix(constraint_matrix);
        solver_.UpdateConstraintBounds(lower_bound, upper_bound);
        jerk_ = solver_.SolveForVector(desired_jerk);

        const VirtualVector governed_acceleration =
            old_acceleration_ + jerk_ * dt_;

        const VirtualVector governed_velocity =
            old_velocity_
            + old_acceleration_ * dt_
            + 0.5 * jerk_ * dt_ * dt_;

        const VirtualVector governed_position =
            old_position_
            + old_velocity_ * dt_
            + 0.5 * old_acceleration_ * dt_ * dt_
            + (1.0 / 6.0) * jerk_ * dt_ * dt_ * dt_;

        model_ptr_->SetState(governed_position, governed_velocity);

        old_position_ = governed_position;
        old_velocity_ = governed_velocity;
        old_acceleration_ = governed_acceleration;
    }

    ConstraintStatus GetConstraintStatus(const size_t& index) const{
        return status_.at(index);
    }

    const VirtualVector& GetAcceleration(void) const{
        return old_acceleration_;
    }

    const VirtualVector& GetJerk(void) const{
        return jerk_;
    }

private:
    struct ConstraintData{
        ConstraintUpdater updater_;
        Scalar max_acceleration_;
        Scalar max_jerk_;
        Scalar activation_tolerance_;
    };

    Scalar dt_;
    DynamicsBase<VirtualDimension, Scalar>* model_ptr_;

    VirtualVector old_position_;
    VirtualVector old_velocity_;
    VirtualVector old_acceleration_;
    VirtualVector jerk_;

    std::array<ConstraintData, MaxConstraints> constraints_;
    std::array<ConstraintStatus, MaxConstraints> status_;
    unsigned number_of_constraints_;

    ClosestVector<VirtualDimension, NumSolverConstraints, Scalar> solver_;
};

}   // namespace constraints
}   // namespace gtfo
