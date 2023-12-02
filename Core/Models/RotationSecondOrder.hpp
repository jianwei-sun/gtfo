//----------------------------------------------------------------------------------------------------
// File: RotationSecondOrder.hpp
// Desc: rotational component of rigid body dynamics
//----------------------------------------------------------------------------------------------------
#pragma once

// Third-party dependencies
#include <Eigen/Geometry>

// Project-specific
#include "DynamicsBase.hpp"
#include "../Utils/UtilityFunctions.hpp"
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
        const Quaternion orientation(Base::position_.data());

        // Integrate angular velocity to get a delta quaternion transformation
        const Scalar half_angle = 0.5 * dt_ * Base::velocity_.norm();
        const Scalar scale = 0.5 * dt_ * sinc(half_angle);
        const Quaternion delta(std::cos(half_angle), Base::velocity_[0] * scale, Base::velocity_[1] * scale, Base::velocity_[2] * scale);
        Base::position_ = (orientation * delta).normalized().coeffs();
    }

    // Calling GetPosition returns the quaternion as a Vector4
    [[nodiscard]] inline Quaternion GetOrientation(void) const{
        return Quaternion(Base::position_.data());
    }

private:
    const Scalar dt_;
    const Matrix3 inertia_;
    const Scalar damping_;
};

}