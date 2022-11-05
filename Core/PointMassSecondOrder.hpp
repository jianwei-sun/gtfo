//----------------------------------------------------------------------------------------------------
// File: PointMassSecondOrder.hpp
// Desc: a second-order dynamics model
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "ModelBase.hpp"

namespace gtfo{

template<typename Scalar = double>
struct SecondOrderParameters : ParametersBase<Scalar>{
    Scalar mass;
    Scalar damping;

    SecondOrderParameters(const Scalar& dt, const Scalar& mass, const Scalar& damping)
        :   ParametersBase<Scalar>(dt), mass(mass), damping(damping)
    {
        assert(mass > 0.0 && damping > 0.0);
    }
};

template<unsigned int Dimensions, typename Scalar = double>
class PointMassSecondOrder : public ModelBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>{
public:
    using Base = ModelBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    PointMassSecondOrder(){}

    void SetParameters(const SecondOrderParameters<Scalar>& parameters) override{
        const Scalar& dt = parameters.dt;
        const Scalar& mass = parameters.mass;
        const Scalar& damping = parameters.damping;

        // Update the continuous-time state transition matrices
        this->A_continuous_ << 0.0, 1.0, 0.0, -damping / mass;
        this->B_continuous_ << 0.0, 1.0 / mass;

        // Update the discrete-time state transition matrices, which are computed using exact discretization
        const Scalar exponent = std::exp(-damping / mass * dt);
        this->A_discrete_ << 1.0, (1.0 - exponent) * mass / damping, 
              0.0, exponent;
        this->B_discrete_ << dt / damping - (1.0 - exponent) * mass / damping,
              (1.0 - exponent) / damping;
    }
};

}   // namespace gtfo
