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

        ParametersBase() : dt(0.001) {}

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

        PointMassBase()
            : A_discrete_(Matrix2::Zero()),
              B_discrete_(Vector2::Zero()),
              position_(VectorN::Zero()),
              velocity_(VectorN::Zero()),
              acceleration_(VectorN::Zero())
        {
            
        }

        virtual void SetParameters(const Parameters &parameters) = 0;

        template <typename BoundType>
        void SetHardBound(const BoundType& bound){
            static_assert(std::is_base_of_v<BoundExpression<Dimensions, Scalar>, BoundType>, "Hard bound must be a BoundExpression or a derived class");
            hard_bound_ = hard_bound_ & bound;
            assert(hard_bound_.Contains(position_));
        }

        // Propagate the dynamics forward by one time-step
        virtual void Step(const VectorN &user_input, const VectorN &environment_input = VectorN::Zero())
        {
            const VectorN total_input = user_input + environment_input;
            const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << position_.transpose(), velocity_.transpose()).finished();
            const Eigen::Matrix<Scalar, 2, Dimensions> new_state = A_discrete_ * state + (total_input * B_discrete_.transpose()).transpose();

            position_ = hard_bound_.GetNearestPointWithinBound(new_state.row(0), state.row(0));
            velocity_ = new_state.row(1);

            if(hard_bound_.IsAtBoundary(position_)){
                const VectorN surface_normal = hard_bound_.GetSurfaceNormal(position_);
                const Scalar inner_product = velocity_.dot(surface_normal);
                if(inner_product > 0.0){
                    velocity_ -= inner_product * surface_normal;
                }
            }

            acceleration_ = (velocity_ - state.row(1).transpose()) / parameters_.dt;
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

    private:
        BoundExpression<Dimensions, Scalar> hard_bound_;
    };

} // namespace gtfo
