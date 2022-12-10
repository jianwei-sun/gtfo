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

    // Propagate the dynamics forward by one time-step
    void Step(const VectorN &user_input, const VectorN &environment_input = VectorN::Zero()) override
    {
        VectorN restorative_soft_bound_force = VectorN::Zero();
        // Check for softbound violations and introduce our restorative force
        if (!this->soft_bound_.Contains(this->position_))
        {
            // TODO impliment this once SoftBoundPhysics is done
            // restorative_soft_bound_force = RestoringForce(this->soft_bound_, this->position_)
        }
        const VectorN total_input = user_input + environment_input + restorative_soft_bound_force;
        const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << this->position_.transpose(), this->velocity_.transpose()).finished();
        Eigen::Matrix<Scalar, 2, Dimensions> new_state = this->A_discrete_ * state + this->B_discrete_ * total_input.transpose();

        this->velocity_ = new_state.row(1);

        // Check for hard bounds and react accordingly
        if (this->hard_bound_.IsAtBoundary(this->position_))
        {
            for (const VectorN &surface_normal : this->hard_bound_.GetSurfaceNormals(this->position_))
            {
                const Scalar inner_product = this->velocity_.dot(surface_normal);
                if (inner_product > 0.0)
                {
                    this->velocity_ -= inner_product * surface_normal;
                }
            }
            // Update the new position with a semi-implicit Euler approximation with the corrected velocity,
            // since the bound-oblivious exact discretization equations would have likely violated the bound
            new_state.row(0) = state.row(0) + this->velocity_.transpose() * this->parameters_.dt;
        }

        this->position_ = this->hard_bound_.GetNearestPointWithinBound(new_state.row(0), state.row(0));
        this->acceleration_ = (this->velocity_ - state.row(1).transpose()) / this->parameters_.dt;
    }
};

}   // namespace gtfo
