//----------------------------------------------------------------------------------------------------
// File: ConstantVelocityModel.hpp
// Desc: Dynamics model for creating a basic constant velocity trajectory use to translate 
//       simple "move-up" and "move-down" commands into references for a controller
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "DynamicsBase.hpp"

namespace gtfo{

template <unsigned int Dimensions, typename Scalar = double>
class ConstantVelocityModel : public DynamicsBase<Dimensions, Scalar>{
public:
    using Base = DynamicsBase<Dimensions, Scalar>;
    using VectorN = typename Base::VectorN;

    ConstantVelocityModel(const Scalar& dt, const Scalar& speed)
        :   Base(VectorN::Zero()),
            dt_(dt),
            // time_(0.0),
            speed_(speed)
    {
        assert(speed_ > 0.0);
    }

    void SyncSystemTo(const Base& model) override{
        Base::position_ = model.position_;
        Base::velocity_ = model.velocity_;
        this->EnforceHardBound();
        Base::dynamics_paused_ = model.dynamics_paused_;
    }

    bool Step(const VectorN& input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        // Hold the current position if dynamics are paused
        if(DynamicsArePaused()){
            Base::velocity_.setZero();
            return true;
        }

        const VectorN directions = (input.array() > 0.0).Select(VectorN::Ones(), (input.array() < 0.0).Select(-VectorN::Ones(), VectorN::Zeros()));

        Base::position_ += (speed_ * dt_) * directions;
        Base::velocity_ = speed_ * directions;
        this->EnforceHardBound();
    }

private:
    const Scalar dt_;
    // Scalar time_;
    const Scalar speed_;
};

}   // namespace gtfo