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

        PointMassBase(const VectorN &initial_physical_position_ = VectorN::Zero())
            : A_discrete_(Matrix2::Zero()),
              B_discrete_(Vector2::Zero()),
              hard_bounded_position_(initial_physical_position_),
              true_position_(initial_physical_position_),
              velocity_(VectorN::Zero()),
              acceleration_(VectorN::Zero())
        {
            
        }

        virtual void SetParameters(const Parameters &parameters) = 0;

        template <typename BoundType>
        void SetHardBound(const BoundType& bound){
            static_assert(std::is_base_of_v<BoundExpression<Dimensions, Scalar>, BoundType>, "Hard bound must be a BoundExpression or a derived class");
            hard_bound_ = hard_bound_ & bound;
            assert(hard_bound_.Contains(hard_bounded_position_));
        }

        // Updates the hard bounded position given a physical position provided by the user
        virtual void UpdateHardBoundedPosition(const VectorN &physical_position)
        {
          if(hard_bound_.IsAtBoundary(hard_bounded_position_))
          {
            const VectorN physical_shift = (this->true_position_ - physical_position); 
            // Uncomment when RemoveBadVelocites function exists
            //this->hard_bounded_position_ = this->hard_bounded_position_ + RemoveBadDirections(physical_shift); 
          }
        
          this->hard_bounded_position_ = hard_bound_.GetNearestPointWithinBound(physical_position,this->hard_bounded_position_);
          this->true_position_ = physical_position; 
        }

        // Propagate the dynamics forward by one time-step. Note force_input is the combined user input plus any virtual environmental effects to be included
        virtual void Step(const VectorN &force_input, const VectorN &physical_position = VectorN::Constant(NAN))
        {
            UpdateHardBoundedPosition(physical_position.array().isNaN().select(hard_bounded_position_,physical_position)); // Update position from the user if given
            const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << hard_bounded_position_.transpose(), velocity_.transpose()).finished();
            Eigen::Matrix<Scalar, 2, Dimensions> new_state = A_discrete_ * state + B_discrete_ * force_input.transpose();

            velocity_ = new_state.row(1);
            if(hard_bound_.IsAtBoundary(hard_bounded_position_)){
                for(const VectorN& surface_normal : hard_bound_.GetSurfaceNormals(hard_bounded_position_)){
                    const Scalar inner_product = velocity_.dot(surface_normal);
                    if(inner_product > 0.0){
                        velocity_ -= inner_product * surface_normal;
                    }
                }
                // Update the new position with a semi-implicit Euler approximation with the corrected velocity, 
                // since the bound-oblivious exact discretization equations would have likely violated the bound
                new_state.row(0) = state.row(0) + velocity_.transpose() * parameters_.dt;
            }

            hard_bounded_position_ = hard_bound_.GetNearestPointWithinBound(new_state.row(0), state.row(0));

            acceleration_ = (velocity_ - state.row(1).transpose()) / parameters_.dt;
        }

        [[nodiscard]] inline const VectorN &GetPosition() const
        {
            return hard_bounded_position_;
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

        VectorN hard_bounded_position_;    // Position of the virtual system which respects hard bounds limitations
        VectorN true_position_;            // Actual position of the system. This can be set by the user each step. If not set, it is always equal to hard_bounded_position_
        VectorN velocity_;
        VectorN acceleration_;

    private:
        BoundExpression<Dimensions, Scalar> hard_bound_;
    };

} // namespace gtfo
