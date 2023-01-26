//----------------------------------------------------------------------------------------------------
// File: PointMassBase.hpp
// Desc: base class for point mass dynamics models
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "DynamicsBase.hpp"

namespace gtfo
{
    template <typename Scalar = double>
    struct ParametersBase
    {
        Scalar dt;

        ParametersBase() : dt(1.0) {}
        ParametersBase(const Scalar &dt) : dt(dt)
        {
            assert(dt > 0.0);
        }
    };

    template <unsigned int Dimensions, typename Parameters, typename Scalar = double>
    class PointMassBase : public DynamicsBase<Dimensions, Scalar>
    {
    public:
        using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
        using DynamicsModelBase = DynamicsBase<Dimensions, Scalar>;

        PointMassBase()
            : DynamicsModelBase(),
              A_discrete_(Eigen::Matrix<Scalar, 2, 2>::Zero()),
              B_discrete_(Eigen::Matrix<Scalar, 2, 1>::Zero()),
              acceleration_(VectorN::Zero())
        {

        }

        virtual void SetParameters(const Parameters &parameters) = 0;

        // Propagate the dynamics forward by one time-step.  Returns false if fails to set a physical position
        virtual bool Step(const VectorN &force_input, const VectorN &physical_position = VectorN::Constant(NAN)) override
        {
            // If we were given a physical location we update our virtual position to match
            const bool err = DynamicsModelBase::SyncVirtualPositionToPhysical(physical_position);

            // Sum the external forces and build the current state
            const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << DynamicsModelBase::position_.transpose(), DynamicsModelBase::velocity_.transpose()).finished();

            // Step the dynamics to determine our next state
            const Eigen::Matrix<Scalar, 2, Dimensions> new_state = A_discrete_ * state + B_discrete_ * force_input.transpose();
            
            // Update position and velocity
            DynamicsModelBase::position_ = new_state.row(0);
            DynamicsModelBase::velocity_ = new_state.row(1);

            // Ensure the position and velocity bounds are satisfied
            this->EnforceHardBound();
            this->EnforceVelocityLimit();

            // Update acceleration for new state
            acceleration_ = (DynamicsModelBase::velocity_ - state.row(1).transpose()) / parameters_.dt;

            // Return error state to user. TODO: Consider converting to int for allowing other error states
            return err;
        }

        [[nodiscard]] inline const VectorN &GetAcceleration() const
        {
            return acceleration_;
        }

    protected:
        Parameters parameters_;

        Eigen::Matrix<Scalar, 2, 2> A_discrete_;
        Eigen::Matrix<Scalar, 2, 1> B_discrete_;

    private:
        VectorN acceleration_;
    };

} // namespace gtfo
