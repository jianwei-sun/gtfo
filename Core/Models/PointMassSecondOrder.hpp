//----------------------------------------------------------------------------------------------------
// File: PointMassSecondOrder.hpp
// Desc: a second-order dynamics model
//----------------------------------------------------------------------------------------------------
#pragma once

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "PointMassBase.hpp"

namespace gtfo{

template<typename Scalar = double>
struct SecondOrderParameters : ParametersBase<Scalar>{
    Scalar mass;
    Scalar damping;

    SecondOrderParameters()
        : ParametersBase<Scalar>(), mass(1.0), damping(1.0) {}

    SecondOrderParameters(const Scalar& dt, const Scalar& mass, const Scalar& damping)
        :   ParametersBase<Scalar>(dt), mass(mass), damping(damping)
    {
        assert(mass > 0.0 && damping > 0.0);
    }
};

template<unsigned int Dimensions, typename Scalar = double>
class PointMassSecondOrder : public PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>{
public:
    using Base = PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    PointMassSecondOrder(){}

    void SetParameters(const SecondOrderParameters<Scalar> &parameters) override
    {
        this->parameters_ = parameters;

        const Scalar& dt = parameters.dt;
        const Scalar& mass = parameters.mass;
        const Scalar& damping = parameters.damping;

        // Update the discrete-time state transition matrices, which are computed using exact discretization
        const Scalar exponent = std::exp(-damping / mass * dt);
        this->A_discrete_ << 1.0, (1.0 - exponent) * mass / damping, 
              0.0, exponent;
        this->B_discrete_ << (damping * dt - (1.0 - exponent) * mass) / (damping * damping),
              (1.0 - exponent) / damping;
    }

    template <typename BoundType>
    void SetSoftBound(const BoundType& bound){
        static_assert(std::is_base_of_v<BoundExpression<Dimensions, Scalar>, BoundType>, "Soft bound must be a BoundExpression or a derived class");
        soft_bound_ = soft_bound_ & bound;
        assert(soft_bound_.Contains(this->position_));
    }

    // Propagate the dynamics forward by one time-step
    void Step(const VectorN &user_input, const VectorN &environment_input = VectorN::Zero()) override
    {
        VectorN restorative_soft_bound_force = VectorN::Zero();
        // Check for softbound violations and introduce our restorative force
        if (!this->soft_bound_.Contains(this->position_))
        {
            // TODO calculate softbound physics here
        }

        // Step the system using the base implementation
        Base::Step(user_input, environment_input + restorative_soft_bound_force);
    }

private:
    BoundExpression<Dimensions, Scalar> soft_bound_;
};

}   // namespace gtfo
