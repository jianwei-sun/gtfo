//----------------------------------------------------------------------------------------------------
// File: PointMassBase.hpp
// Desc: base class for dynamics models
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <cmath>
#include <type_traits>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Bounds/BoundBase.hpp"

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
    class PointMassBase
    {
    public:
        static_assert(Dimensions > 0, "Dimensions must be at least 1.");
        static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");

        using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
        using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
        using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;

        using BoundPtr = std::shared_ptr<BoundBase<Dimensions, Scalar>>;

        PointMassBase()
            : A_discrete_(Matrix2::Zero()),
              B_discrete_(Vector2::Zero()),
              position_(VectorN::Zero()),
              velocity_(VectorN::Zero()),
              acceleration_(VectorN::Zero())
        {
            hard_bound_ = std::make_shared<BoundBase<Dimensions, Scalar>>();
        }

        virtual void SetParameters(const Parameters &parameters) = 0;

        template <typename BoundType>
        void SetHardBound(const BoundType& bound){
            static_assert(
                std::is_base_of_v<BoundBase<Dimensions, Scalar>, BoundType>, 
                "Hard bound must derive from BoundBase"
            );
            hard_bound_ = std::make_shared<BoundType>(bound);
            assert(hard_bound_->Contains(position_));
        }

        // Propagate the dynamics forward by one time-step.  Returns false if fails to set a physical position
        virtual bool Step(const VectorN &force_input, const VectorN &physical_position = VectorN::Constant(NAN))
        {
            // If we were given a physical location we update our virtual position to match
            bool err = PointMassBase::SyncVirtualPositionToPhysical(physical_position);

            // Sum the external forces and build the current state
            const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << position_.transpose(), velocity_.transpose()).finished();

            // Step the dynamics to determine our next state
            Eigen::Matrix<Scalar, 2, Dimensions> new_state = A_discrete_ * state + B_discrete_ * force_input.transpose();

            // Ensure the new position is within the bounds
            position_ = hard_bound_->GetNearestPointWithinBound(new_state.row(0));
            
            // Ensure the new velocity points in the interior of the bound
            velocity_ = new_state.row(1);

            // Enfoce hard bounds if they exist and then update the state position
            EnforceHardBound(state, new_state);

            // Update acceleration for new state
            acceleration_ = (velocity_ - state.row(1).transpose()) / parameters_.dt;

            // Return error state to user. TODO: Consider converting to int for allowing other error states
            return err;
        }

        [[nodiscard]] inline const VectorN &GetPosition() const
        {
            return position_;
        }

        [[nodiscard]] inline const VectorN &GetVelocity() const
        {
            return velocity_;
        }

        [[nodiscard]] inline const VectorN &GetAcceleration() const
        {
            return acceleration_;
        }

    protected:
        Parameters parameters_;

        Matrix2 A_discrete_;
        Vector2 B_discrete_;

        VectorN position_;
        VectorN velocity_;
        VectorN acceleration_;

        BoundPtr hard_bound_;

        // Sets the virtual position to the physical one if the physical input is in hard bounds.
        virtual bool SyncVirtualPositionToPhysical(const VectorN &physical_position)
        {
            if (!physical_position.array().isNaN().any())
            {
                // Snap position to closest point to physical position in bounds.
                position_ = hard_bound_->GetNearestPointWithinBound(physical_position);

                // Let user know if snap worked
               return !hard_bound_->Contains(physical_position);
            }

            return true;
        }

    private:
        // Enforce hard bound limits on a state to determine position new state
        virtual void EnforceHardBound(const Eigen::Matrix<Scalar, 2, Dimensions> &state, Eigen::Matrix<Scalar, 2, Dimensions> &new_state)
        {
            const auto surface_normals = hard_bound_->GetSurfaceNormals(position_);
            if (surface_normals.HasPositiveDotProductWith(velocity_))
            {
                surface_normals.RemoveComponentIn(velocity_);
                new_state.row(0) = state.row(0) + velocity_.transpose() * parameters_.dt;
            }
            position_ = hard_bound_->GetNearestPointWithinBound(new_state.row(0));
        }
    };

} // namespace gtfo
