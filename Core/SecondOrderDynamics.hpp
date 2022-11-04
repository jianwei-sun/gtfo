//----------------------------------------------------------------------------------------------------
// File: SecondOrderDynamics.hpp
// Desc: a second-order dynamics model
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "DynamicsBase.hpp"

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
class SecondOrderDynamics : public DynamicsBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>{
public:
    using Base = DynamicsBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    SecondOrderDynamics() 
        :   acceleration_(VectorN::Zero()){}

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

    void Step(const VectorN& input) override{
        acceleration_ = this->A_continuous_.row(1) * (Eigen::Matrix<Scalar, 2, Dimensions>() << this->position_.transpose(), this->velocity_.transpose()).finished() + this->B_continuous_.row(1) * input;
        Base::Step(input);
    }

private:
    VectorN acceleration_;
};

}   // namespace gtfo
