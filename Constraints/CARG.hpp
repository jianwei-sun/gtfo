//----------------------------------------------------------------------------------------------------
// File: CARG.hpp
// Desc: Capture-admissible reference governor
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
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

    // Reserve five rows per geometric constraint: a zero-authority capture
    // row, the combined normal-jerk interval, an endpoint acceleration row, and the positive-
    // part transition's zero-branch row, and an endpoint capture-margin row.
    static constexpr unsigned NumSolverConstraints = 5 * MaxConstraints;

    using VirtualVector = Eigen::Matrix<Scalar, VirtualDimension, 1>;
    using ConstraintMatrix = Eigen::Matrix<Scalar, NumSolverConstraints, VirtualDimension>;
    using ConstraintVector = Eigen::Matrix<Scalar, NumSolverConstraints, 1>;

    enum class UpdateStatus { NotRun, Success, NoModel, SolverFailure };

    UpdateStatus GetUpdateStatus() const{ return update_status_; }
    c_int GetSolveStatus() const{ return solver_.GetSolveStatus(); }
    // Optional exponential barrier: dot(g) >= -rate*g.
    // Zero retains the original nondecreasing-margin policy.
    void SetCaptureMarginRate(Scalar rate){
        if(!std::isfinite(rate) || rate < 0) throw std::invalid_argument("Invalid capture margin rate");
        capture_margin_rate_ = rate;
    }
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
            assert(max_jerk(i) >= 0);
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

        // A projection for one boundary can endanger another boundary that
        // was safe under the nominal command. Grow the active set from the
        // governed candidate before committing any state. Every retry adds
        // at least one constraint. Endpoint corrections also have a fixed budget.
        std::array<bool, MaxConstraints> additionally_active{};
        std::array<bool, MaxConstraints> acceleration_cut{};
        std::array<VirtualVector, MaxConstraints> acceleration_gradient;
        std::array<Scalar, MaxConstraints> acceleration_offset{};
        std::array<bool, MaxConstraints> capture_cut{};
        std::array<VirtualVector, MaxConstraints> capture_gradient;
        std::array<Scalar, MaxConstraints> capture_offset{};
        Scalar margin_rate = capture_margin_rate_;
        // Reserve bounded retries for nonlinear endpoint acceleration correction.
        for(unsigned pass = 0; pass <= number_of_constraints_ + 12; ++pass){
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
                    additionally_active[i]
                    || status.capture_margin_ <= constraints_[i].activation_tolerance_
                    || status.predicted_capture_margin_ <= constraints_[i].activation_tolerance_;

                if(!status.active_){
                    continue;
                }

                const unsigned row = 5 * i;

                if(acceleration_cut[i]){
                    constraint_matrix.row(row + 2) = acceleration_gradient[i].transpose();
                    lower_bound(row + 2) = -constraints_[i].max_acceleration_ - acceleration_offset[i];
                    upper_bound(row + 2) = constraints_[i].max_acceleration_ - acceleration_offset[i];
                }
                if(capture_cut[i]){
                    constraint_matrix.row(row + 4) = capture_gradient[i].transpose();
                    lower_bound(row + 4) = -capture_offset[i];
                }

                // These three inequalities all bound the same normal jerk
                // j_i = gamma_i * u + beta_i. Intersect their scalar intervals
                // before handing them to OSQP. Separate antiparallel rows become
                // numerically troublesome when capture requires exactly -j_bar.
                Scalar jerk_lower = std::max(-constraints_[i].max_jerk_,
                    (-constraints_[i].max_acceleration_ - old_constraint.acceleration_) / dt_);
                Scalar jerk_upper = std::min(constraints_[i].max_jerk_,
                    (constraints_[i].max_acceleration_ - old_constraint.acceleration_) / dt_);
                const Scalar capture_rhs = -old_constraint.velocity_
                    - capture.Dv_ * old_constraint.acceleration_
                    + margin_rate * std::max(Scalar(0), status.capture_margin_);
                if(capture.Da_ > Scalar(0)){
                    // For both braking branches in ComputeCaptureDistance,
                    // -v - Dv*a = -j_bar*Da analytically. Use that identity
                    // instead of dividing two quantities that vanish near rest.
                    jerk_upper = std::min(jerk_upper, -constraints_[i].max_jerk_
                        + margin_rate * std::max(Scalar(0), status.capture_margin_) / capture.Da_);
                } else {
                    // No control authority in this row; retain a negative RHS
                    // as an infeasible zero row rather than silently dropping it.
                    upper_bound(row) = capture_rhs;
                }

                // Algebraically equal endpoints can differ by a few ulps after
                // capture differentiation. Reconcile only roundoff-sized gaps.
                const Scalar roundoff = Scalar(64) * std::numeric_limits<Scalar>::epsilon()
                    * std::max({Scalar(1), std::abs(jerk_lower), std::abs(jerk_upper)});
                if(std::isfinite(jerk_lower) && std::isfinite(jerk_upper) &&
                   jerk_lower > jerk_upper && jerk_lower - jerk_upper <= roundoff){
                    jerk_lower = jerk_upper = (jerk_lower + jerk_upper) / Scalar(2);
                }
                if(jerk_lower > jerk_upper){
                    // OSQP's bounds-update API rejects l > u. Encode a genuine
                    // empty interval as 0 <= -1 so the solver can report infeasible.
                    upper_bound(row + 1) = Scalar(-1);
                } else {
                    constraint_matrix.row(row + 1) = old_constraint.gamma_.transpose();
                    lower_bound(row + 1) = jerk_lower - old_constraint.beta_;
                    upper_bound(row + 1) = jerk_upper - old_constraint.beta_;
                }

                // At D=max(0,chi)=0, dot(D)=max(0,dot(chi)). Both branches
                // must obey dot(D)<=-v; the excursion branch is the row above.
                if(capture.zero_excursion_transition_)
                    upper_bound(row + 3) = -old_constraint.velocity_
                        + margin_rate * std::max(Scalar(0), status.capture_margin_);

            }

            // Geometry coefficients can have very different magnitudes.
            // ClosestVector sets up OSQP before these coefficients are available,
            // so retain explicit positive row scaling, which preserves the QP.
            for(unsigned row = 0; row < NumSolverConstraints; ++row){
                const Scalar norm = constraint_matrix.row(row).stableNorm();
                if(norm > 0 && std::isfinite(norm)){
                    constraint_matrix.row(row) /= norm;
                    if(lower_bound(row) > -OSQP_INFTY) lower_bound(row) /= norm;
                    if(upper_bound(row) < OSQP_INFTY) upper_bound(row) /= norm;
                }
                // Keep zero rows, including infeasible zero-branch constraints.
            }

            // Opposite faces of a corridor describe the same control direction.
            // Intersect duplicate normalized rows across geometric constraints,
            // too; leaving a tight bound beside its redundant opposite can stall
            // ADMM even after a fresh setup.
            for(unsigned row = 0; row < NumSolverConstraints; ++row){
                if(constraint_matrix.row(row).squaredNorm() == 0) continue;
                for(unsigned other = row + 1; other < NumSolverConstraints; ++other){
                    const Scalar sign = constraint_matrix.row(row).dot(constraint_matrix.row(other)) >= 0 ? Scalar(1) : Scalar(-1);
                    if((constraint_matrix.row(row) - sign*constraint_matrix.row(other)).norm()
                        > Scalar(64)*std::numeric_limits<Scalar>::epsilon()) continue;
                    const Scalar lo = sign > 0 ? lower_bound(other) : -upper_bound(other);
                    const Scalar hi = sign > 0 ? upper_bound(other) : -lower_bound(other);
                    lower_bound(row) = std::max(lower_bound(row), lo);
                    upper_bound(row) = std::min(upper_bound(row), hi);
                    constraint_matrix.row(other).setZero();
                    lower_bound(other) = -OSQP_INFTY;
                    upper_bound(other) = OSQP_INFTY;
                }
                if(lower_bound(row) > upper_bound(row)){
                    constraint_matrix.row(row).setZero();
                    lower_bound(row) = -OSQP_INFTY;
                    upper_bound(row) = Scalar(-1);
                }
            }

            solver_.UpdateConstraintMatrix(constraint_matrix);
            solver_.UpdateConstraintBounds(lower_bound, upper_bound);

            // Equivalent unconstrained solution of the acceleration-tracking objective.
            VirtualVector candidate_jerk = solver_.SolveForVector(
                (nominal_acceleration - old_acceleration_) / dt_);
            if(!solver_.LastSolveSucceeded() &&
               (solver_.GetSolveStatus() == OSQP_MAX_ITER_REACHED ||
                solver_.GetSolveStatus() == OSQP_SOLVED_INACCURATE)){
                // Rebuild scaling from the current matrix and discard stale
                // dual iterates once. The QP, tolerances and limits are identical.
                // ClosestVector's copy constructor performs a fresh OSQP setup.
                auto restarted = solver_;
                swap(solver_, restarted);
                candidate_jerk = solver_.SolveForVector(
                    (nominal_acceleration - old_acceleration_) / dt_);
            }
            if(!solver_.LastSolveSucceeded()){
                // A fixed exponential decay rate can make opposite faces
                // incompatible inside the safe set. Allow faster use of the
                // available margin, at most its one-step Euler budget. Actual
                // acceleration, jerk and endpoint admissibility remain hard.
                if(margin_rate > 0 && margin_rate < Scalar(1)/dt_){
                    margin_rate = std::min(Scalar(2)*margin_rate, Scalar(1)/dt_);
                    acceleration_cut.fill(false);
                    capture_cut.fill(false);
                    continue;
                }
                update_status_ = UpdateStatus::SolverFailure;
                return false;
            }

            // Constant-jerk propagation from governed state k-1 to governed state k.
            const VirtualVector governed_acceleration =
                old_acceleration_ + candidate_jerk * dt_;

            const VirtualVector governed_velocity =
                old_velocity_
                + old_acceleration_ * dt_
                + 0.5 * candidate_jerk * dt_ * dt_;

            const VirtualVector governed_position =
                old_position_
                + old_velocity_ * dt_
                + 0.5 * old_acceleration_ * dt_ * dt_
                + (1.0 / 6.0) * candidate_jerk * dt_ * dt_ * dt_;

            bool retry = false;
            for(unsigned i = 0; i < number_of_constraints_; ++i){
                Constraint governed_constraint;
                constraints_[i].updater_(governed_constraint, governed_position,
                    governed_velocity, governed_acceleration, current_time);
                const auto governed_capture = ComputeCaptureDistance(
                    governed_constraint.velocity_, governed_constraint.acceleration_,
                    constraints_[i].max_acceleration_, constraints_[i].max_jerk_);
                const Scalar governed_margin = governed_constraint.clearance_ - governed_capture.distance_;
                const bool bad_acceleration = std::abs(governed_constraint.acceleration_) > constraints_[i].max_acceleration_ + Scalar(1e-8);
                const bool bad_capture = governed_margin < Scalar(-1e-9);
                // The old-state jerk row predicts acceleration only to first
                // order. Re-linearize the actual integrated endpoint if changing
                // geometry invalidates that prediction. Never commit that state.
                if(bad_acceleration || bad_capture){
                    for(unsigned j = 0; j < VirtualDimension; ++j){
                        const Scalar step = Scalar(1e-3) * std::max(Scalar(1), std::abs(candidate_jerk(j)));
                        VirtualVector dq = VirtualVector::Zero();
                        dq(j) = step;
                        Constraint plus, minus;
                        constraints_[i].updater_(plus,
                            governed_position + dq * (dt_*dt_*dt_/Scalar(6)),
                            governed_velocity + dq * (dt_*dt_/Scalar(2)),
                            governed_acceleration + dq * dt_, current_time);
                        constraints_[i].updater_(minus,
                            governed_position - dq * (dt_*dt_*dt_/Scalar(6)),
                            governed_velocity - dq * (dt_*dt_/Scalar(2)),
                            governed_acceleration - dq * dt_, current_time);
                        if(bad_acceleration)
                            acceleration_gradient[i](j) = (plus.acceleration_ - minus.acceleration_) / (Scalar(2)*step);
                        if(bad_capture){
                            const Scalar gp = plus.clearance_ - ComputeCaptureDistance(plus.velocity_, plus.acceleration_, constraints_[i].max_acceleration_, constraints_[i].max_jerk_).distance_;
                            const Scalar gm = minus.clearance_ - ComputeCaptureDistance(minus.velocity_, minus.acceleration_, constraints_[i].max_acceleration_, constraints_[i].max_jerk_).distance_;
                            capture_gradient[i](j) = (gp-gm)/(Scalar(2)*step);
                        }
                    }
                    if(bad_acceleration){
                        acceleration_offset[i] = governed_constraint.acceleration_
                            - acceleration_gradient[i].dot(candidate_jerk);
                        acceleration_cut[i] = true;
                    }
                    if(bad_capture){
                        capture_offset[i] = governed_margin - capture_gradient[i].dot(candidate_jerk);
                        capture_cut[i] = true;
                    }
                    additionally_active[i] = true;
                    retry = true;
                }
                if(status_[i].active_) continue;
                // Preserve the same early-braking band after projection that
                // is used before projection. Waiting for a negative margin
                // lets a coupled correction consume almost the entire reserve
                // in one step, leaving no room for discrete geometry error.
                if(governed_constraint.clearance_ - governed_capture.distance_ <= constraints_[i].activation_tolerance_
                    || std::abs(governed_constraint.acceleration_) > constraints_[i].max_acceleration_){
                    additionally_active[i] = true;
                    retry = true;
                }
            }
            if(retry) continue;

            model_ptr_->SetState(governed_position, governed_velocity, governed_acceleration);

            jerk_ = candidate_jerk;
            old_position_ = governed_position;
            old_velocity_ = governed_velocity;
            old_acceleration_ = governed_acceleration;
            update_status_ = UpdateStatus::Success;
            return true;
        }
        // Nonlinear corrections may fail to converge within the fixed budget.
        // Retain the previously accepted state rather than commit that candidate.
        update_status_ = UpdateStatus::SolverFailure;
        return false;
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
    Scalar capture_margin_rate_ = 0;
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
