//----------------------------------------------------------------------------------------------------
// File: PointMassFirstOrder.hpp
// Desc: a first-order dynamics model
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "PointMassBase.hpp"

namespace gtfo{

template<typename Scalar = double>
struct FirstOrderParameters : ParametersBase<Scalar>{
    Scalar time_constant;
    Scalar dc_gain;

    FirstOrderParameters(const Scalar& dt, const Scalar& time_constant, const Scalar& dc_gain)
        :   ParametersBase<Scalar>(dt), time_constant(time_constant), dc_gain(dc_gain)
    {
        assert(time_constant > 0.0 && dc_gain > 0.0);
    }
};

template<unsigned int Dimensions, typename Scalar = double>
class PointMassFirstOrder : public PointMassBase<Dimensions, FirstOrderParameters<Scalar>, Scalar>{
public:
    using Base = PointMassBase<Dimensions, FirstOrderParameters<Scalar>, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using PossibleScalar = std::conditional_t<Dimensions == 1, Scalar, VectorN>;

    PointMassFirstOrder(){}

    void SetParameters(const FirstOrderParameters<Scalar>& parameters) override{
        const Scalar& dt = parameters.dt;
        const Scalar& dc_gain = parameters.dc_gain;
        const Scalar& time_constant = parameters.time_constant;

        // Update the continuous-time state transition matrices
        this->A_continuous_ << -1.0 / time_constant, 0.0, 0.0, 0.0;
        this->B_continuous_ << dc_gain / time_constant, 0.0;

        // Update the discrete-time state transition matrices, which are computed using exact discretization
        const Scalar exponent = std::exp(-dt / time_constant);
        this->A_discrete_ << exponent, 0.0, 0.0, 1.0;
        this->B_discrete_ << (1.0 - exponent) * dc_gain, 0.0;
    }

    void Step(const PossibleScalar& input) override{
        this->velocity_ = this->A_continuous_(0,0) * this->position_ + input * this->B_continuous_.row(0);
        Base::Step(input);
    }
};

}   // namespace gtfo
