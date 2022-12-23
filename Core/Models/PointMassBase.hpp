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
              hard_virtual_position_(initial_physical_position_),
              physical_position_(initial_physical_position_),
              velocity_(VectorN::Zero()),
              acceleration_(VectorN::Zero())
        {
            
        }

        virtual void SetParameters(const Parameters &parameters) = 0;

        template <typename BoundType>
        void SetHardBound(const BoundType& bound){
            static_assert(std::is_base_of_v<BoundExpression<Dimensions, Scalar>, BoundType>, "Hard bound must be a BoundExpression or a derived class");
            hard_bound_ = hard_bound_ & bound;
            assert(hard_bound_.Contains(hard_virtual_position_));
        }

        // Upddates the hard virtual position given a physical position provided by the user
        virtual void UpdateHardVirtualPosition(const VectorN &physical_position)
        {
          if(hard_bound_.IsAtBoundary(hard_virtual_position_))
          {
            const VectorN physical_velocity = (this->physical_position_ - physical_position) / this->parameters_.dt; 
            // Uncomment when RemoveBadVelocites function exists
            //this->hard_virtual_position_ = this->hard_virtual_position_ + RemoveBadVelocities(physical_velocity) * this->parameters_.dt; 
          }
        
          this->hard_virtual_position_ = hard_bound_.GetNearestPointWithinBound(physical_position,this->hard_virtual_position_);
          this->physical_position_ = physical_position; 
        }

        // Propagate the dynamics forward by one time-step
        virtual void Step(const VectorN &user_input, const VectorN &environment_input = VectorN::Zero(), const VectorN &physical_position = hard_virtual_position_)
        {
            UpdateHardVirtualPosition(physical_position); // Update position from the user
            const VectorN total_input = user_input + environment_input;
            const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << hard_virtual_position_.transpose(), velocity_.transpose()).finished();
            Eigen::Matrix<Scalar, 2, Dimensions> new_state = A_discrete_ * state + B_discrete_ * total_input.transpose();

            velocity_ = new_state.row(1);
            if(hard_bound_.IsAtBoundary(hard_virtual_position_)){
                for(const VectorN& surface_normal : hard_bound_.GetSurfaceNormals(hard_virtual_position_)){
                    const Scalar inner_product = velocity_.dot(surface_normal);
                    if(inner_product > 0.0){
                        velocity_ -= inner_product * surface_normal;
                    }
                }
                // Update the new position with a semi-implicit Euler approximation with the corrected velocity, 
                // since the bound-oblivious exact discretization equations would have likely violated the bound
                new_state.row(0) = state.row(0) + velocity_.transpose() * parameters_.dt;
            }

            hard_virtual_position_ = hard_bound_.GetNearestPointWithinBound(new_state.row(0), state.row(0));

            acceleration_ = (velocity_ - state.row(1).transpose()) / parameters_.dt;
        }

        [[nodiscard]] inline const VectorN &GetPosition() const
        {
            return hard_virtual_position_;
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

        VectorN hard_virtual_position_;    // Position of the virtual system which cannoy exceede hard bounds
        VectorN physical_position_;        // Position of the physical system updated by user.  If no update is given this is always equal to hard_virtual_position_
        VectorN velocity_;
        VectorN acceleration_;

    private:
        BoundExpression<Dimensions, Scalar> hard_bound_;
    };

} // namespace gtfo
