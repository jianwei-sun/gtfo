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
    };

    template <unsigned int Dimensions, typename Scalar = double>
    class PointMassSecondOrder : public PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>
    {
    public:
        using Base = PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>;
        using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

        PointMassSecondOrder(const VectorN& initial_position = VectorN::Zero())
            : Base(initial_position) 
        {
            
        }

        void SetParameters(const SecondOrderParameters<Scalar> &parameters) override
        {
            Base::parameters_ = parameters;

            const Scalar &dt = parameters.dt;
            const Scalar &mass = parameters.mass;
            const Scalar &damping = parameters.damping;

            // Update the discrete-time state transition matrices, which are computed using exact discretization
            const Scalar exponent = std::exp(-damping / mass * dt);
            Base::A_discrete_ << 1.0, (1.0 - exponent) * mass / damping, 
                0.0, exponent;
            Base::B_discrete_ << (damping * dt - (1.0 - exponent) * mass) / (damping * damping),
                (1.0 - exponent) / damping;
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

    };

} // namespace gtfo
