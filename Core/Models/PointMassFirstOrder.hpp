//----------------------------------------------------------------------------------------------------
// File: PointMassFirstOrder.hpp
// Desc: a first-order dynamics model
//       This essentially serves as a low-pass filter. I.e., if a constant "force" is input to the
//       system, its position will asymptotically equal the force * dc gain
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "PointMassBase.hpp"

namespace gtfo{

template<typename Scalar = double>
struct FirstOrderParameters : ParametersBase<Scalar>{
    Scalar time_constant;
    Scalar dc_gain;

    FirstOrderParameters()
        :   ParametersBase<Scalar>(), time_constant(1.0), dc_gain(1.0) {}

    FirstOrderParameters(const Scalar& dt, const Scalar& time_constant, const Scalar& dc_gain)
        :   ParametersBase<Scalar>(dt), time_constant(time_constant), dc_gain(dc_gain)
    {
        assert(time_constant > 0.0 && dc_gain > 0.0);
    }

    FirstOrderParameters operator+(const FirstOrderParameters& other){
        return FirstOrderParameters(ParametersBase<Scalar>::dt, 
            time_constant + other.time_constant, 
            dc_gain + other.dc_gain);
    }

    FirstOrderParameters operator*(const Scalar& scalar){
        return FirstOrderParameters(ParametersBase<Scalar>::dt, 
            scalar * time_constant, 
            scalar * dc_gain);
    }
};

template<unsigned int Dimensions, typename Scalar = double>
class PointMassFirstOrder : public PointMassBase<Dimensions, FirstOrderParameters<Scalar>, Scalar>{
public:
    using Base = PointMassBase<Dimensions, FirstOrderParameters<Scalar>, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    PointMassFirstOrder(const FirstOrderParameters<Scalar> &parameters, const VectorN &initial_position = VectorN::Zero())
        : Base(parameters, initial_position)
    {
        SetStateTransitionMatrices(parameters);
    }

private:
    void SetStateTransitionMatrices(const FirstOrderParameters<Scalar> &parameters) override
    {
        const Scalar& dt = parameters.dt;
        const Scalar& dc_gain = parameters.dc_gain;
        const Scalar& time_constant = parameters.time_constant;

        // Update the discrete-time state transition matrices, which are computed using exact discretization
        const Scalar exponent = std::exp(-dt / time_constant);
        Base::A_discrete_ << exponent, 0.0, -1.0 / time_constant, 0.0;
        Base::B_discrete_ << (1.0 - exponent) * dc_gain, dc_gain / time_constant;
    }
};

}   // namespace gtfo
