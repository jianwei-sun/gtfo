//----------------------------------------------------------------------------------------------------
// File: PointMassSecondOrder.hpp
// Desc: a second-order dynamics model
//----------------------------------------------------------------------------------------------------
#pragma once
#include <cmath>
// Project-specific
#include "PointMassBase.hpp"

namespace gtfo{

    template <typename Scalar = double>
    struct SecondOrderParameters : ParametersBase<Scalar>
    {
        Scalar mass;
        Scalar damping;
        Scalar stiffness;

        SecondOrderParameters()
            : ParametersBase<Scalar>(), mass(1.0), damping(1.0), stiffness(1.0)
        {
        }

        SecondOrderParameters(const Scalar &dt, const Scalar &mass, const Scalar &damping, const Scalar &stiffness)
            : ParametersBase<Scalar>(dt), mass(mass), damping(damping), stiffness(stiffness)
        {
            assert(mass > 0.0 && damping > 0.0 && stiffness >= 0.0);
        }

        SecondOrderParameters operator+(const SecondOrderParameters& other){
            return SecondOrderParameters(ParametersBase<Scalar>::dt, 
                mass + other.mass, 
                damping + other.damping,
                stiffness + other.stiffness);
        }

        SecondOrderParameters operator*(const Scalar& scalar){
            return SecondOrderParameters(ParametersBase<Scalar>::dt, 
                scalar * mass, 
                scalar * damping,
                scalar * stiffness);
        }
    };

    template <unsigned int Dimensions, typename Scalar = double>
    class PointMassSecondOrder : public PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>
    {
    public:
        using Base = PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>;
        using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

        PointMassSecondOrder(const SecondOrderParameters<Scalar> &parameters, const VectorN &initial_position = VectorN::Zero())
            : Base(parameters, initial_position),
            initial_position_(initial_position)
        {
            SetStateTransitionMatrices(parameters);
        }

        // Propagate dynamics for a second order system but using softbounds if they exist
        void PropagateDynamics(const VectorN &force_input) override
        {
            Base::PropagateDynamics(force_input);
            
            // Calculate the acceleration using the more accurate continuous equations with the current velocity
            Base::acceleration_ = (-Base::parameters_.damping / Base::parameters_.mass) * Base::velocity_ + (-Base::parameters_.stiffness \
            / Base:: parameters_.mass) * (Base::position_ - this->initial_position_) +force_input / Base::parameters_.mass;
        }

    private:
        VectorN initial_position_;
        void SetStateTransitionMatrices(const SecondOrderParameters<Scalar> &parameters) override
        {
            const Scalar &dt = parameters.dt;
            const Scalar &mass = parameters.mass;
            const Scalar &damping = parameters.damping;
            const Scalar &stiffness = parameters.stiffness;

            // Update the discrete-time state transition matrices, which are computed using exact discretization
            if (stiffness == 0){
                const Scalar exponent = std::exp(-damping / mass * dt);
                Base::A_discrete_ << static_cast<Scalar>(1.0), (static_cast<Scalar>(1.0) - exponent) * mass / damping,
                    static_cast<Scalar>(0.0), exponent;
                Base::B_discrete_ << (damping * dt - (static_cast<Scalar>(1.0) - exponent) * mass) / (damping * damping),
                    (static_cast<Scalar>(1.0) - exponent) / damping;}
            else {
            // here we used a first order approximation
                Base::A_discrete_ << static_cast<Scalar>(1.0), dt,
                    - (dt * stiffness) / mass, static_cast<Scalar>(1.0) - (damping * dt) / mass;
                Base::B_discrete_ << static_cast<Scalar>(0.0),
                    dt/mass;
                }
            Base::C_discrete_ << static_cast<Scalar>(0.0), stiffness;
        }
    };

} // namespace gtfo
