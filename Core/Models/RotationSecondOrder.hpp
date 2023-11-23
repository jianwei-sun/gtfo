//----------------------------------------------------------------------------------------------------
// File: RotationSecondOrder.hpp
// Desc: rotational component of rigid body dynamics
//----------------------------------------------------------------------------------------------------
#pragma once

// Third-party dependencies
#include <Eigen/Geometry>

// Project-specific
#include "DynamicsBase.hpp"

namespace gtfo
{

template<typename Scalar = double>
class RotationSecondOrder : public DynamicsBase<3, Scalar, 4>{
public:
    using Base = DynamicsBase<3, Scalar, 4>;
    using Vector3 = typename Base::VectorN;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Quaternion = Eigen::Quaternion<Scalar>;

    RotationSecondOrder(const Scalar& dt, const Vector3& principal_inertia, const Scalar& damping, const Quaternion& initial_orientation = Quaternion::Identity())
        :   Base(initial_orientation.coeffs()),
            dt_(dt),
            inertia_(principal_inertia.asDiagonal()),
            damping_(damping)
    {
        assert(dt_ > 0.0);

        // Positive definiteness
        assert((principal_inertia.array() > 0.0).all());

        // Triangle inequality
        assert(principal_inertia[0] <= principal_inertia[1] + principal_inertia[2]);
        assert(principal_inertia[1] <= principal_inertia[2] + principal_inertia[0]);
        assert(principal_inertia[2] <= principal_inertia[0] + principal_inertia[1]);

        // Passivity
        assert(damping_ >= 0.0);
    }

    // Torque is in the body frame
    void PropagateDynamics(const Vector3& torque) override{
        // Compute and store the constant
        static const Matrix3 inertia_inverse = inertia_.inverse();

        // Propagate the body frame velocity with Euler's equations
        Base::velocity_ = (Base::velocity_ + dt_ * inertia_inverse * (
            -(Base::velocity_.cross(inertia_ * Base::velocity_))
            -damping_ * Base::velocity_
            + torque
        )).eval();

        // Current position interpreted as an orientation
        Quaternion& orientation = Eigen::Map<Quaternion>(Base::position_.data());

        // Assume the angular velocity is constant over the period and integrate it to get a delta position.
        // Then, apply the delta transformation to update the body orientation
        const Scalar speed = Base::velocity_.norm();
        if(speed >= GTFO_EQUALITY_COMPARISON_TOLERANCE){
            const Scalar angle = 0.5 * speed * dt_;
            const Scalar scale = std::sin(angle) / speed;
            const Quaternion delta(std::cos(angle), scale * Base::velocity_[0], scale * Base::velocity_[1], scale * Base::velocity_[2]);
            orientation = (delta * orientation).normalized().eval();
        }

        // The following coarser approximation may work well for small timestamps.
        // TODO: compare the two approaches
        // orientation = (orientation + 0.5 * orientation * Base::velocity_ * dt_).normalized().eval();
    }

    // Calling GetPosition returns the quaternion as a Vector4
    [[nodiscard]] inline Quaternion GetOrientation(void) const{
        return Eigen::Map<Quaternion>(Base::position_.data());
    }

private:
    const Scalar dt_;
    const Matrix3 inertia_;
    const Scalar damping_;
};

}