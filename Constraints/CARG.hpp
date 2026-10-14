//----------------------------------------------------------------------------------------------------
// File: CARG.hpp
// Desc: Capture-admissible reference governor
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <array>
#include <cassert>
#include <cstddef>
#include <functional>
#include <stdexcept>

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

    // Each geometric constraint contributes:
    // 1) excursion-branch invariance, 2) normal jerk, 3) normal acceleration,
    // 4) zero-branch invariance at a positive-part transition.
    static constexpr unsigned NumSolverConstraints = 4 * MaxConstraints;

    using VirtualVector = Eigen::Matrix<Scalar, VirtualDimension, 1>;
    using ConstraintMatrix = Eigen::Matrix<Scalar, NumSolverConstraints, VirtualDimension>;
    using ConstraintVector = Eigen::Matrix<Scalar, NumSolverConstraints, 1>;

    enum class UpdateStatus { NotRun, Success, NoModel, SolverFailure };

    UpdateStatus GetUpdateStatus() const{ return update_status_; }
    c_int GetSolveStatus() const{ return solver_.GetSolveStatus(); }
    bool SetSolverTolerances(const Scalar& absolute, const Scalar& relative, c_int max_iterations){
        return solver_.SetSolverTolerances(absolute, relative, max_iterations);
    }

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

    // Static geometry.
    using ConstraintUpdater =
        std::function<void(
            Constraint&,
            const VirtualVector&,
            const VirtualVector&,
            const VirtualVector&)>;

    // Time-varying geometry. The last argument is the evaluation time.
    using TimeVaryingConstraintUpdater =
        std::function<void(
            Constraint&,
            const VirtualVector&,
            const VirtualVector&,
            const VirtualVector&,
            const Scalar&)>;

    explicit CARG(const Scalar& dt)
        :   dt_(dt),
            model_ptr_(nullptr),
            old_position_(VirtualVector::Zero()),
            old_velocity_(VirtualVector::Zero()),
            old_acceleration_(VirtualVector::Zero()),
            jerk_(VirtualVector::Zero()),
            number_of_constraints_(0)
    {
        assert(dt_ > 0);
    }

    void SetVirtualDynamics(
        DynamicsBase<VirtualDimension, Scalar>* model_ptr,
        const VirtualVector& initial_acceleration = VirtualVector::Zero())
    {
        assert(model_ptr);

        model_ptr_ = model_ptr;
        old_position_ = model_ptr_->GetPosition();
        old_velocity_ = model_ptr_->GetVelocity();
        old_acceleration_ = initial_acceleration;
        jerk_.setZero();
        update_status_ = UpdateStatus::NotRun;
    }

    // Add a static geometric constraint.
    void AddConstraint(
        const ConstraintUpdater& updater,
        const Scalar& max_acceleration,
        const Scalar& max_jerk,
        const Scalar& activation_tolerance = 0.0)
    {
        assert(static_cast<bool>(updater));

        AddTimeVaryingConstraint(
            [updater](
                Constraint& constraint,
                const VirtualVector& position,
                const VirtualVector& velocity,
                const VirtualVector& acceleration,
                const Scalar&)
            {
                updater(constraint, position, velocity, acceleration);
            },
            max_acceleration,
            max_jerk,
            activation_tolerance);
    }

    // Add a geometric constraint whose clearance geometry varies with time.
    void AddTimeVaryingConstraint(
        const TimeVaryingConstraintUpdater& updater,
        const Scalar& max_acceleration,
        const Scalar& max_jerk,
        const Scalar& activation_tolerance = 0.0)
    {
        assert(number_of_constraints_ < MaxConstraints);
        assert(static_cast<bool>(updater));
        assert(max_acceleration > 0);
        assert(max_jerk > 0);
        assert(activation_tolerance >= 0);

        ConstraintData& data = constraints_[number_of_constraints_++];
        data.updater_ = updater;
        data.max_acceleration_ = max_acceleration;
        data.max_jerk_ = max_jerk;
        data.activation_tolerance_ = activation_tolerance;
    }

    void ClearConstraints(void){
        number_of_constraints_ = 0;
    }

    // Generalized jerk bounds: -u_bar <= u <= u_bar.
    void SetVirtualJerkLimits(const VirtualVector& max_jerk){
        for(unsigned i = 0; i < VirtualDimension; ++i){
            assert(max_jerk(i) > 0);
            solver_.SetDimensionBounds(i, -max_jerk(i), max_jerk(i));
        }
    }

    // Call after the admittance model has generated the current nominal candidate at current_time.
    void UpdateVirtualState(const Scalar& current_time){
        if(!TryUpdateVirtualState(current_time) && update_status_ == UpdateStatus::SolverFailure)
            throw std::runtime_error("CARG: QP failed; nominal reference left unchanged and not accepted");
    }

    // Nonthrowing solver-failure path. On solver failure, leave the model's nominal
    // reference untouched and retain the previous accepted state and jerk internally.
    // The nominal reference is NOT accepted or certified by CARG. The caller must
    // handle failure before the next control step; retrying this update uses the same
    // current_time and previous accepted state.
    bool TryUpdateVirtualState(const Scalar& current_time){
        if(!model_ptr_){
            update_status_ = UpdateStatus::NoModel;
            return false;
        }

        // Current unconstrained admittance candidate at k.
        const VirtualVector nominal_position = model_ptr_->GetPosition();
        const VirtualVector nominal_velocity = model_ptr_->GetVelocity();
        const VirtualVector nominal_acceleration = model_ptr_->GetAcceleration();

        ConstraintMatrix constraint_matrix = ConstraintMatrix::Zero();
        ConstraintVector lower_bound =
            ConstraintVector::Constant(-OSQP_INFTY);
        ConstraintVector upper_bound =
            ConstraintVector::Constant(OSQP_INFTY);

        const Scalar old_time = current_time - dt_;

        for(unsigned i = 0; i < number_of_constraints_; ++i){
            Constraint old_constraint;
            Constraint nominal_constraint;

            // Previous accepted governed state at k-1 and its geometry.
            constraints_[i].updater_(
                old_constraint,
                old_position_,
                old_velocity_,
                old_acceleration_,
                old_time);

            // Current unconstrained candidate at k and its geometry.
            constraints_[i].updater_(
                nominal_constraint,
                nominal_position,
                nominal_velocity,
                nominal_acceleration,
                current_time);

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

            const unsigned row = 4 * i;

            // dot(g_i) >= 0
            constraint_matrix.row(row) =
                (capture.Da_ * old_constraint.gamma_).transpose();

            upper_bound(row) =
                -old_constraint.velocity_
                - capture.Dv_ * old_constraint.acceleration_
                - capture.Da_ * old_constraint.beta_;

            // At D=max(0,chi)=0, dot(D)=max(0,dot(chi)). Both branches
            // must obey dot(D)<=-v; the excursion branch is the row above.
            if(capture.zero_excursion_transition_)
                upper_bound(row + 3) = -old_constraint.velocity_;

            // |j_i| <= j_bar_i
            constraint_matrix.row(row + 1) =
                old_constraint.gamma_.transpose();

            lower_bound(row + 1) =
                -constraints_[i].max_jerk_
                - old_constraint.beta_;

            upper_bound(row + 1) =
                constraints_[i].max_jerk_
                - old_constraint.beta_;

            // |a_i + dt * j_i| <= a_bar_i
            constraint_matrix.row(row + 2) =
                old_constraint.gamma_.transpose();

            lower_bound(row + 2) =
                (-constraints_[i].max_acceleration_
                 - old_constraint.acceleration_) / dt_
                - old_constraint.beta_;

            upper_bound(row + 2) =
                (constraints_[i].max_acceleration_
                 - old_constraint.acceleration_) / dt_
                - old_constraint.beta_;
        }

        solver_.UpdateConstraintMatrix(constraint_matrix);
        solver_.UpdateConstraintBounds(lower_bound, upper_bound);

        // Equivalent unconstrained solution of the acceleration-tracking objective.
        const VirtualVector candidate_jerk = solver_.SolveForVector(
            (nominal_acceleration - old_acceleration_) / dt_);
        if(!solver_.LastSolveSucceeded()){
            update_status_ = UpdateStatus::SolverFailure;
            return false;
        }
        jerk_ = candidate_jerk;

        // Constant-jerk propagation from governed state k-1 to governed state k.
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

        model_ptr_->SetState(governed_position, governed_velocity, governed_acceleration);

        old_position_ = governed_position;
        old_velocity_ = governed_velocity;
        old_acceleration_ = governed_acceleration;
        update_status_ = UpdateStatus::Success;
        return true;
    }

    ConstraintStatus GetConstraintStatus(const size_t& index) const{
        assert(index < number_of_constraints_);
        return status_[index];
    }

    const VirtualVector& GetAcceleration(void) const{
        return old_acceleration_;
    }

    const VirtualVector& GetJerk(void) const{
        return jerk_;
    }

private:
    struct ConstraintData{
        TimeVaryingConstraintUpdater updater_;
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
    UpdateStatus update_status_ = UpdateStatus::NotRun;

    ClosestVector<VirtualDimension, NumSolverConstraints, Scalar> solver_;
};

}   // namespace constraints
}   // namespace gtfo
