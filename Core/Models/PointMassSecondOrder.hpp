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
        using BoundPtr = std::shared_ptr<BoundBase<Dimensions, Scalar>>;

        PointMassSecondOrder()
        {
            soft_bound_ = nullptr;
        }

        void SetParameters(const SecondOrderParameters<Scalar> &parameters) override
        {
            this->parameters_ = parameters;

            const Scalar &dt = parameters.dt;
            const Scalar &mass = parameters.mass;
            const Scalar &damping = parameters.damping;

            // Update the discrete-time state transition matrices, which are computed using exact discretization
            const Scalar exponent = std::exp(-damping / mass * dt);
            this->A_discrete_ << 1.0, (1.0 - exponent) * mass / damping,
                0.0, exponent;
            this->B_discrete_ << (damping * dt - (1.0 - exponent) * mass) / (damping * damping),
                (1.0 - exponent) / damping;
        }

        template <typename BoundType>
        void SetSoftBound(const BoundType &bound, const Scalar &spring_constant)
        {
            static_assert(std::is_base_of_v<BoundBase<Dimensions, Scalar>, BoundType>, "Soft bound must derive from BoundBase");
            soft_bound_ = std::make_shared<BoundType>(bound);
            assert(soft_bound_->Contains(this->position_));
            soft_bound_spring_constant = spring_constant;
        }

        // Propagate dynamics for a second order system but using softbounds if they exist
        void Step(const VectorN &user_input, const VectorN &environment_input = VectorN::Zero()) override
        {
            // Apply softbound logic if a softbound exists to determine restoring force on the next state
            VectorN soft_bound_restoring_force = VectorN::Zero();
            if (soft_bound_)
            {
                const VectorN total_input = user_input + environment_input;
                const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << this->position_.transpose(), this->velocity_.transpose()).finished();
                Eigen::Matrix<Scalar, 2, Dimensions> next_state = this->A_discrete_ * state + this->B_discrete_ * total_input.transpose();
                soft_bound_restoring_force = EnforceSoftBound(next_state);
            }

            // Step using the base logic
            PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>::Step(user_input, environment_input + soft_bound_restoring_force);
        }

    private:
        BoundPtr soft_bound_;
        Scalar soft_bound_spring_constant;

        // Enfoce soft bound limits on a state to determine the restoring force
        VectorN EnforceSoftBound(Eigen::Matrix<Scalar, 2, Dimensions> &next_state)
        {
            VectorN soft_bound_restoring_force = VectorN::Zero();

            // If we are outside bounds we need to add a restoring force
            if (!soft_bound_->Contains(next_state.row(0)))
            {
                const VectorN temp_position = next_state.row(0);
                const VectorN temp_velocity = next_state.row(1);
                const auto surface_normals = soft_bound_->GetSurfaceNormals(soft_bound_->GetNearestPointWithinBound(temp_position));

                // If we are still trying to move outward
                if (surface_normals.HasPositiveDotProductWith(temp_velocity))
                {
                    // Add a restoring force in directions that we are pushing away from bounds
                    for (const VectorN &surface_normal : surface_normals)
                    {
                        for (unsigned i = 0; i < temp_velocity.size(); i++)
                        {
                            VectorN single_direction_velocity = VectorN::Zero();
                            single_direction_velocity(i) = temp_velocity(i);
                            if (single_direction_velocity.dot(surface_normal) > 0.0)
                            {
                                soft_bound_restoring_force(i) = soft_bound_restoring_force(i) + soft_bound_spring_constant * (soft_bound_->GetNearestPointWithinBound(temp_position)(i) - temp_position(i));
                            }
                        }
                    }
                }
            }

            return soft_bound_restoring_force;
        }
    };

} // namespace gtfo
