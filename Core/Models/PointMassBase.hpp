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

namespace gtfo
{

    template <typename Scalar = double>
    struct ParametersBase
    {
        Scalar dt;

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

        void SetPositionBound(const Bound<Scalar, Dimensions> &position_bound)
        {
            position_bound_ = position_bound;
        }

        void SetVelocityBound(const Bound<Scalar, Dimensions> &velocity_bound)
        {
            velocity_bound_ = velocity_bound;
        }

        void SetUserInputBound(const Bound<Scalar, Dimensions> &user_input_bound)
        {
            user_input_bound_ = user_input_bound;
        }

        // Propagate the dynamics forward by one time-step
        virtual void Step(const VectorN &user_input, const VectorN &environment_input = VectorN::Zero())
        {
            const VectorN total_input = user_input_bound_(user_input) + environment_input;
            const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << position_.transpose(), velocity_.transpose()).finished();
            const Eigen::Matrix<Scalar, 2, Dimensions> new_state = A_discrete_ * state + (total_input * B_discrete_.transpose()).transpose();

            position_ = position_bound_(new_state.row(0));
            velocity_ = velocity_bound_(new_state.row(1));
            acceleration_ = (velocity_ - state.row(1)) / parameters_.dt;
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
        Bound<Scalar, Dimensions> position_bound_;
        Bound<Scalar, Dimensions> velocity_bound_;
        Bound<Scalar, Dimensions> user_input_bound_;
    };

} // namespace gtfo
