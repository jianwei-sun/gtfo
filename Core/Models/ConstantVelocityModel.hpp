//----------------------------------------------------------------------------------------------------
// File: ConstantVelocityModel.hpp
// Desc: Dynamics model for creating a basic constant velocity trajectory use to translate 
//       simple "move-up" and "move-down" commands into references for a controller
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "DynamicsBase.hpp"
#include "../Utils/Comparisons.hpp"

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

    void PropagateDynamics(const VectorN& direction) override{
        // Hold the current position if dynamics are paused or the input is zero
        if(IsEqual(direction, VectorN::Zero())){
            Base::velocity_.setZero();
            return;
        }
        Base::position_ += (speed_ * dt_) * direction.normalized();
        Base::velocity_ = speed_ * direction.normalized();
        Base::acceleration_.setZero();
    }

private:
    const Scalar dt_;
    const Scalar speed_;
};

}   // namespace gtfo