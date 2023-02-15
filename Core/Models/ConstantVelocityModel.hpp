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
            speed_(speed)
    {
        assert(speed_ > 0.0);
    }

    void SyncSystemTo(const Base& model) override{
        Base::position_ = model.GetPosition();
        Base::velocity_ = model.GetVelocity();
        this->EnforceHardBound();
        Base::dynamics_paused_ = model.DynamicsArePaused();
    }

    bool Step(const VectorN& input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        // Hold the current position if dynamics are paused
        if(Base::DynamicsArePaused()){
            Base::velocity_.setZero();
            return true;
        }

        const VectorN directions = input.array().sign();
        Base::position_ += (speed_ * dt_) * directions;
        Base::velocity_ = speed_ * directions;
        this->EnforceHardBound();
        return true;
    }

private:
    const Scalar dt_;
    const Scalar speed_;
};

}   // namespace gtfo