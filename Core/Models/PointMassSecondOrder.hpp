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
            soft_bound_ = std::make_shared<BoundBase<Dimensions, Scalar>>();
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
        void SetSoftBound(const BoundType &bound, const Scalar &spring_constant, const Scalar &damping_constant)
        {
            static_assert(std::is_base_of_v<BoundBase<Dimensions, Scalar>, BoundType>, "Soft bound must derive from BoundBase");
            soft_bound_ = std::make_shared<BoundType>(bound);
            assert(soft_bound_->Contains(this->position_));
            soft_bound_spring_constant = spring_constant;
            soft_bound_damping_constant = damping_constant;
        }

        // Propagate dynamics for a second order system but using softbounds if they exist
        void Step(const VectorN &user_input, const VectorN &environment_input = VectorN::Zero()) override
        {
            // Check to first see if we are outside a softbound
            if (!soft_bound_->Contains(this->position_))
            {
                // If we are outside we need to enforce the softbound and use its restoring force to step
                VectorN soft_bound_restoring_force = EnforceSoftBound();
                PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>::Step(user_input, environment_input + soft_bound_restoring_force);
                return;
            }

            // Otherwise step without a restoring force
            PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>::Step(user_input, environment_input);
        }

    private:
        BoundPtr soft_bound_;
        Scalar soft_bound_spring_constant;
        Scalar soft_bound_damping_constant;

        // Enfoce soft bound limits on a state to determine the restoring force
        VectorN EnforceSoftBound()
        {
            // Default to no restoring force
            VectorN soft_bound_restoring_force = VectorN::Zero();

            // Find our surface normals to so we can see if we are moving twoards bounds or away
            const auto surface_normals = soft_bound_->GetSurfaceNormals(soft_bound_->GetNearestPointWithinBound(this->position_));

            // Add a spring force based on how far we are outside the bounds
            soft_bound_restoring_force = -soft_bound_spring_constant * (this->position_ - soft_bound_->GetNearestPointWithinBound(this->position_));

            // If we are trying to move outward we also add a damping force in directions that we are pushing away from bounds
            if (surface_normals.HasPositiveDotProductWith(this->velocity_))
            {
                for (const VectorN &surface_normal : surface_normals)
                {
                    for (unsigned i = 0; i < this->velocity_.size(); i++)
                    {
                        VectorN single_direction_velocity = VectorN::Zero();
                        single_direction_velocity(i) = this->velocity_(i);
                        if (single_direction_velocity.dot(surface_normal) > 0.0)
                        {
                            soft_bound_restoring_force(i) = soft_bound_restoring_force(i) + soft_bound_damping_constant * -this->velocity_(i);
                        }
                    }
                }
            }

            return soft_bound_restoring_force;
        }
    };

} // namespace gtfo
