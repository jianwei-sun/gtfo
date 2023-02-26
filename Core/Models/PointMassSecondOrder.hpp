//----------------------------------------------------------------------------------------------------
// File: PointMassSecondOrder.hpp
// Desc: a second-order dynamics model
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "PointMassBase.hpp"

namespace gtfo{

    template <typename Scalar = double>
    struct SecondOrderParameters : ParametersBase<Scalar>
    {
        Scalar mass;
        Scalar damping;

        SecondOrderParameters()
            : ParametersBase<Scalar>(), mass(1.0), damping(1.0)
        {
        }

        SecondOrderParameters(const Scalar &dt, const Scalar &mass, const Scalar &damping)
            : ParametersBase<Scalar>(dt), mass(mass), damping(damping)
        {
            assert(mass > 0.0 && damping > 0.0);
        }

        SecondOrderParameters operator+(const SecondOrderParameters& other){
            return SecondOrderParameters(ParametersBase<Scalar>::dt, 
                mass + other.mass, 
                damping + other.damping);
        }

        SecondOrderParameters operator*(const Scalar& scalar){
            return SecondOrderParameters(ParametersBase<Scalar>::dt, 
                scalar * mass, 
                scalar * damping);
        }
    };

    template <unsigned int Dimensions, typename Scalar = double>
    class PointMassSecondOrder : public PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>
    {
    public:
        using Base = PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>;
        using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

        PointMassSecondOrder(const SecondOrderParameters<Scalar> &parameters, const VectorN &initial_position = VectorN::Zero())
            : Base(parameters, initial_position)
        {
            SetStateTransitionMatrices(parameters);
        }

        // Propagate dynamics for a second order system but using softbounds if they exist
        bool Step(const VectorN &force_input, const VectorN &physical_position = VectorN::Constant(NAN)) override
        {
            // If we were given a physical location we update our virtual position to match
            const bool err = Base::SyncVirtualPositionToPhysical(physical_position);

            // If we are outside we need to enforce the softbound and use its restoring force to step without a physical position since we already synced it (if it was valid)
            const VectorN soft_bound_restoring_force = this->EnforceSoftBound();

            // Otherwise step without a restoring force and also without a physical position since we already synced it (if it was valid)
            Base::Step(force_input + soft_bound_restoring_force);

            return err;
        }

    private:
        void SetStateTransitionMatrices(const SecondOrderParameters<Scalar> &parameters) override
        {
            const Scalar &dt = parameters.dt;
            const Scalar &mass = parameters.mass;
            const Scalar &damping = parameters.damping;

            // Update the discrete-time state transition matrices, which are computed using exact discretization
            const Scalar exponent = std::exp(-damping / mass * dt);
            Base::A_discrete_ << static_cast<Scalar>(1.0), (static_cast<Scalar>(1.0) - exponent) * mass / damping,
                static_cast<Scalar>(0.0), exponent;
            Base::B_discrete_ << (damping * dt - (static_cast<Scalar>(1.0) - exponent) * mass) / (damping * damping),
                (static_cast<Scalar>(1.0) - exponent) / damping;
        }

        // Calculate the acceleration using the continuous equations with the current velocity
        void UpdateAcceleration(const VectorN& force_input, const VectorN& previous_velocity) override{
            Base::acceleration_ = (-Base::parameters_.damping / Base::parameters_.mass) * Base::velocity_ + force_input / Base::parameters_.mass;
        }
    };

} // namespace gtfo
