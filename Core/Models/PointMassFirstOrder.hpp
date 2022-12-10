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
};

template<unsigned int Dimensions, typename Scalar = double>
class PointMassFirstOrder : public PointMassBase<Dimensions, FirstOrderParameters<Scalar>, Scalar>{
public:
    using Base = PointMassBase<Dimensions, FirstOrderParameters<Scalar>, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    PointMassFirstOrder(){}

    void SetParameters(const FirstOrderParameters<Scalar>& parameters) override{
        this->parameters_ = parameters;

        const Scalar& dt = parameters.dt;
        const Scalar& dc_gain = parameters.dc_gain;
        const Scalar& time_constant = parameters.time_constant;

        // Update the discrete-time state transition matrices, which are computed using exact discretization
        const Scalar exponent = std::exp(-dt / time_constant);
        this->A_discrete_ << exponent, 0.0, -1.0 / time_constant, 0.0;
        this->B_discrete_ << (1.0 - exponent) * dc_gain, dc_gain / time_constant;
    }

    // Propagate the dynamics forward by one time-step
    void Step(const VectorN &user_input, const VectorN &environment_input = VectorN::Zero()) override
    {
        const VectorN total_input = user_input + environment_input;
        const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << this->position_.transpose(), this->velocity_.transpose()).finished();
        Eigen::Matrix<Scalar, 2, Dimensions> new_state = this->A_discrete_ * state + this->B_discrete_ * total_input.transpose();

        this->velocity_ = new_state.row(1);

        // Check for hard bounds react accordingly
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
