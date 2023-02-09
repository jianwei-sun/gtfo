//----------------------------------------------------------------------------------------------------
// File: DynamicsBase.hpp
// Desc: base class for dynamics models
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <memory>
#include <type_traits>

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
            velocity_(VectorN::Zero()),
            acceleration_(VectorN::Zero()),
            dynamics_paused_(false),
            hard_bound_(new BoundBase<Dimensions, Scalar>()),
            soft_bound_(new BoundBase<Dimensions, Scalar>()),
            soft_bound_spring_constant_(0.0),
            soft_bound_damping_constant_(0.0),
            velocity_bound_(new BoundBase<Dimensions, Scalar>())
    {
        
    }

    // Sets the current model's state to that of the target model. Since the current model may have different
    // bounds than the target model, the updated state is modified to satisfy the bounds. This may result in
    // discontinuities in the state if the target state is out of bounds
    template <typename Model>
    void SyncSystemTo(const Model& model){
        static_assert(std::is_base_of_v<DynamicsBase, Model>, "Model must be a derived class of DynamicsBase");
        static_assert(Dimensions == Model::Dimension, "Model dimension must equal current dimension");
        static_assert(std::is_same_v<Scalar, typename Model::ScalarType>, "ScalarTypes must be equal");

        position_ = model.position_;
        velocity_ = model.velocity_;
        this->EnforceHardBound();
        this->EnforceVelocityLimit();
        acceleration_ = model.acceleration_;
        dynamics_paused_ = model.dynamics_paused_;
    }

    // Pure virtual function to be implemented by the subclass. The function should
    // update position and velocity using the inputs, and enforce bounds if necessary
    // The function should also pause dynamics by referring to the dynamics_paused_ flag
    virtual bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) = 0;

    virtual void PauseDynamics(const bool& pause){
        dynamics_paused_ = pause;
    }

    [[nodiscard]] inline bool DynamicsArePaused(void){
        return dynamics_paused_;
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

    // Hard bound logic modifies the position_ and velocity_ member variables. This function
    // should be called after the position and velocity have been updated by Step
    virtual void EnforceHardBound(void){
        position_ = hard_bound_->GetNearestPointWithinBound(position_);
        const auto surface_normals = hard_bound_->GetSurfaceNormals(position_);
        if(surface_normals.HasPositiveDotProductWith(velocity_)){
            surface_normals.RemoveComponentIn(velocity_);
        }
    }

    template <typename BoundType>
    void SetSoftBound(const BoundType& bound, const Scalar &spring_constant, const Scalar &damping_constant){
        static_assert(
            std::is_base_of_v<BoundBase<Dimensions, Scalar>, BoundType>, 
            "Soft bound must derive from BoundBase"
        );
        soft_bound_ = std::make_shared<BoundType>(bound);
        assert(soft_bound_->Contains(position_));
        soft_bound_spring_constant_ = spring_constant;
        soft_bound_damping_constant_ = damping_constant;
    }

    // Since the soft bound only computes a restoring force, it should not modify the position
    // or velocity states. It should typically be called before Step updates position and
    // velocity so that the restoring force can be used in Step
    [[nodiscard]] virtual VectorN EnforceSoftBound(void) const{
        // Default to no restoring force
        VectorN soft_bound_restoring_force = VectorN::Zero();

        // Find your surface normals to so we can see if we are moving towards bounds or away
        const auto surface_normals = soft_bound_->GetSurfaceNormals(soft_bound_->GetNearestPointWithinBound(position_));

        // Add a spring force based on how far we are outside the bounds
        soft_bound_restoring_force = -soft_bound_spring_constant_ * (position_ - soft_bound_->GetNearestPointWithinBound(position_));

        // If we are trying to move outward we also add a damping force in directions that we are pushing away from bounds
        if (surface_normals.HasPositiveDotProductWith(velocity_))
        {
            for(const VectorN& surface_normal : surface_normals)
            {
                const Scalar dot_product = surface_normal.dot(velocity_);
                if(dot_product > 0.0){
                    soft_bound_restoring_force += -soft_bound_damping_constant_ * dot_product * surface_normal;
                }
            }
        }
        return soft_bound_restoring_force;
    }

    // Sets a norm-bound for the velocity
    void SetVelocityLimit(const Scalar& limit){
        assert(limit >= 0.0);
        velocity_bound_ = std::make_shared<NormBound<Dimensions, Scalar>>(limit);
    }

    // Modifies the current velocity to the closest point within the velocity bound
    virtual void EnforceVelocityLimit(void){
        velocity_ = velocity_bound_->GetNearestPointWithinBound(velocity_);
    }

    // Sets the virtual position to the physical one if the physical input is in hard bounds.
    virtual bool SyncVirtualPositionToPhysical(const VectorN &physical_position)
    {
        if (!physical_position.array().isNaN().any())
        {
            // Snap position to closest point to physical position in bounds.
            position_ = hard_bound_->GetNearestPointWithinBound(physical_position);

            // Let user know if snap worked
            return !hard_bound_->Contains(physical_position);
        }

        return true;
    }

    [[nodiscard]] inline const VectorN &GetPosition() const
    {
        return position_;
    }

    [[nodiscard]] inline const VectorN &GetVelocity() const
    {
        return velocity_;
    }

    [[nodiscard]] inline const VectorN &GetAcceleration() const
    {
        return acceleration_;
    }

protected:
    // Addition states can be added by subclasses, but they should handle their updating
    VectorN position_;
    VectorN velocity_;
    VectorN acceleration_;

    bool dynamics_paused_;
private:
    // Hard and soft bounds are included for convenience, but do not have to be used
    BoundPtr hard_bound_;
    BoundPtr soft_bound_;
    Scalar soft_bound_spring_constant_;
    Scalar soft_bound_damping_constant_;

    // Same goes for velocity limit
    BoundPtr velocity_bound_;
};

}