//----------------------------------------------------------------------------------------------------
// File: DynamicsBase.hpp
// Desc: base class for dynamics models
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <memory>
#include <type_traits>
#include <utility>
#include <functional>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Bounds/BoundBase.hpp"
#include "../Bounds/NormBound.hpp"

namespace gtfo{

template<unsigned int Dimensions, typename Scalar = double>
class DynamicsBase{
public:
    static_assert(Dimensions > 0, "Dimensions must be at least 1.");
    static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using BoundPtr = std::shared_ptr<BoundBase<Dimensions, Scalar>>;

    const static unsigned int Dimension = Dimensions; 
    using ScalarType = Scalar;

    DynamicsBase(const VectorN& initial_position = VectorN::Zero())
        :   position_(initial_position),
            old_position_(VectorN::Zero()),
            velocity_(VectorN::Zero()),
            acceleration_(VectorN::Zero()),
            dynamics_paused_(false),
            soft_bound_restoring_force_(VectorN::Zero()),
            hard_bound_(new BoundBase<Dimensions, Scalar>()),
            soft_bound_(new BoundBase<Dimensions, Scalar>()),
            soft_bound_spring_constant_(0.0),
            soft_bound_damping_constant_(0.0),
            velocity_bound_(new BoundBase<Dimensions, Scalar>()),
            force_premodifier_(nullptr)
    {}

    // Virtual function to be implemented by the subclass. The function should
    // update the position, velocity, and acceleration states
    virtual void PropagateDynamics(const VectorN& force_input) = 0;

    virtual void Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)){
        // Store a history of the position
        old_position_ = position_;

        // Update the position with the physical position, if necessary
        if(!physical_position.array().isNaN().any()){
            position_ = physical_position;
        }

        // Update force, if necessary
        const VectorN modified_force = PremodifyForce(force_input);

        // Dynamics are only propagated when unpaused
        if(dynamics_paused_){
            // When paused, acceleration is also zeroed to prevent an impulse, even though the actual instantaneous acceleration is -velocity / dt
            velocity_.setZero();
            acceleration_.setZero();
        } else{            
            const VectorN soft_bound_restoring_force = this->EnforceSoftBound();
            this->PropagateDynamics(modified_force + soft_bound_restoring_force);
        }

        // Perform the dynamically discontinuous actions, such as enforcing hard bounds
        EnforceStateConstraints();
    }

    //----------------------------------------------------------------------------------------------------
    [[nodiscard]] virtual inline VectorN GetPosition(void) const{
        return position_;
    }

    [[nodiscard]] virtual inline VectorN GetOldPosition(void) const{
        return old_position_;
    }

    [[nodiscard]] virtual inline VectorN GetVelocity(void) const{
        return velocity_;
    }

    [[nodiscard]] virtual inline VectorN GetAcceleration(void) const{
        return acceleration_;
    }

    [[nodiscard]] virtual inline bool DynamicsArePaused(void) const{
        return dynamics_paused_;
    }

    [[nodiscard]] virtual inline VectorN GetSoftBoundRestoringForce(void) const{
        return soft_bound_restoring_force_;
    }

    // Sets the current model's state to that of the target model. Since the current model may have different
    // bounds than the target model, the updated state is modified to satisfy the bounds. This may result in
    // discontinuities in the state if the target state is out of bounds
    virtual void SyncModelTo(const DynamicsBase& model){
        SetFullState(
            model.GetPosition(),
            model.GetOldPosition(),
            model.GetVelocity(),
            model.GetAcceleration(),
            model.DynamicsArePaused(),
            model.GetSoftBoundRestoringForce()
        );
    }

    virtual void SetFullState(const VectorN& position, const VectorN& old_position, const VectorN& velocity, const VectorN& acceleration, const bool& dynamics_paused, const VectorN& soft_bound_restoring_force){
        position_ = position;
        old_position_ = old_position;
        velocity_ = velocity;
        acceleration_ = acceleration;
        dynamics_paused_ = dynamics_paused;
        soft_bound_restoring_force_ = soft_bound_restoring_force;
        EnforceStateConstraints();
    }

    virtual void SetState(const VectorN& position, const VectorN& velocity, const VectorN& acceleration = VectorN::Zero()){
        position_ = position;
        velocity_ = velocity;
        acceleration_ = acceleration;
        EnforceStateConstraints();
    }

    virtual void PauseDynamics(const bool& pause){
        dynamics_paused_ = pause;
    }

    //----------------------------------------------------------------------------------------------------
    // Main function for enforcing all hard limits for states, which may result in discontinuous dynamics
    // TODO: Update with collision avoidance constraints
    void EnforceStateConstraints(void){
        EnforceHardBound();
        EnforceVelocityLimit();
    }

    template <typename BoundType>
    void SetHardBound(const BoundType& bound){
        static_assert(
            std::is_base_of_v<BoundBase<Dimensions, Scalar>, BoundType>, 
            "Hard bound must derive from BoundBase"
        );
        hard_bound_ = std::make_shared<BoundType>(bound);
        assert(hard_bound_->Contains(position_));
    }

    // Hard bound logic modifies position_, velocity_, and acceleration_. This function
    // should be called after the variables have been updated by Step
    void EnforceHardBound(void){
        position_ = hard_bound_->GetNearestPointWithinBound(position_);
        const auto surface_normals = hard_bound_->GetSurfaceNormals(position_);
        if(surface_normals.HasPositiveDotProductWith(velocity_)){
            surface_normals.RemoveComponentIn(velocity_);

            // Also remove acceleration components that try to make velocity point out the bound
            if(surface_normals.HasPositiveDotProductWith(acceleration_)){
                surface_normals.RemoveComponentIn(acceleration_);
            }
        }
    }

    template <typename BoundType>
    void SetSoftBound(const BoundType& bound, const Scalar &spring_constant, const Scalar &damping_constant){
        static_assert(
            std::is_base_of_v<BoundBase<Dimensions, Scalar>, BoundType>, 
            "Soft bound must derive from BoundBase"
        );
        soft_bound_ = std::make_shared<BoundType>(bound);
        soft_bound_spring_constant_ = spring_constant;
        soft_bound_damping_constant_ = damping_constant;
    }

    // Since the soft bound only computes a restoring force, it should not modify the position
    // or velocity states. It should typically be called before Step updates position and
    // velocity so that the restoring force can be used in Step
    [[nodiscard]] VectorN EnforceSoftBound(void){
        // Find your surface normals to so we can see if we are moving towards bounds or away
        const auto surface_normals = soft_bound_->GetSurfaceNormals(soft_bound_->GetNearestPointWithinBound(position_));

        // Add a spring force based on how far we are outside the bounds
        soft_bound_restoring_force_ = -soft_bound_spring_constant_ * (position_ - soft_bound_->GetNearestPointWithinBound(position_));

        // If we are trying to move outward we also add a damping force in directions that we are pushing away from bounds
        if (surface_normals.HasPositiveDotProductWith(velocity_))
        {
            for(const VectorN& surface_normal : surface_normals)
            {
                const Scalar dot_product = surface_normal.dot(velocity_);
                if(dot_product > 0.0){
                    soft_bound_restoring_force_ += -soft_bound_damping_constant_ * dot_product * surface_normal;
                }
            }
        }
        return soft_bound_restoring_force_;
    }

    // Sets a norm-bound for the velocity
    void SetVelocityLimit(const Scalar& limit){
        assert(limit >= 0.0);
        velocity_bound_ = std::make_shared<NormBound<Dimensions, Scalar>>(limit);
    }

    // Modifies the current velocity to the closest point within the velocity bound, and prevent
    // accelerations that try to exceed the velocity bound
    void EnforceVelocityLimit(void){
        velocity_ = velocity_bound_->GetNearestPointWithinBound(velocity_);
        const auto surface_normals = velocity_bound_->GetSurfaceNormals(velocity_);
        if(surface_normals.HasPositiveDotProductWith(acceleration_)){
            surface_normals.RemoveComponentIn(acceleration_);
        }
    }

    void SetForcePremodifier(const std::function<VectorN(const VectorN&, const DynamicsBase<Dimensions, Scalar>&)>& force_premodifier){
        force_premodifier_ = force_premodifier;
    }

    VectorN PremodifyForce(const VectorN& force_input) {
        if(force_premodifier_){
            return force_premodifier_(force_input, *this);
        } else{
            return force_input;
        }
    }

protected:
    // Addition states can be added by subclasses, but they should handle their updating
    VectorN position_;
    VectorN old_position_;
    VectorN velocity_;
    VectorN acceleration_;

    bool dynamics_paused_;
    VectorN soft_bound_restoring_force_;
private:
    // Hard and soft bounds are included for convenience, but do not have to be used
    BoundPtr hard_bound_;
    BoundPtr soft_bound_;
    Scalar soft_bound_spring_constant_;
    Scalar soft_bound_damping_constant_;

    // Same goes for velocity limit
    BoundPtr velocity_bound_;

    // Lambda for force premodifier
    std::function<VectorN(const VectorN&, const DynamicsBase<Dimensions, Scalar>&)> force_premodifier_;
};

}


