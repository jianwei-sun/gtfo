//----------------------------------------------------------------------------------------------------
// File: RigidBodySecondOrder.hpp
// Desc: Full rigid body dynamics
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "DynamicsBase.hpp"
#include "PointMassSecondOrder.hpp"
#include "RotationSecondOrder.hpp"
#include "../Containers/DynamicsVector.hpp"

namespace gtfo
{

template<typename Scalar = double>
class Pose{
public:
    using Position = Eigen::Matrix<Scalar, 3, 1>;
    using Orientation = Eigen::Quaternion<Scalar>;

    Pose(const Position& position, const Orientation& orientation)
        :   position_(position),
            orientation_(orientation)
    {}

    Pose(const Eigen::Matrix<Scalar, 7, 1>& coeffs)
        :   position_(coeffs.template head<3>()),
            orientation_(coeffs.template tail<4>())
    {}

    static Pose Identity(void){
        return Pose(Position::Zero(), Orientation::Identity());
    }
    
    [[nodiscard]] Position GetPosition(void) const{
        return position_;
    }

    [[nodiscard]] Orientation GetOrientation(void) const{
        return orientation_;
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 4, 4> ToMatrix(void) const{
        return (Eigen::Matrix<Scalar, 4, 4>() <<
            orientation_.toRotationMatrix(), position_.transpose(),
            Eigen::RowVector<Scalar, 4>::UnitW()
        ).finished();
    }

    [[nodiscard]] Eigen::Matrix<Scalar, 7, 1> coeffs(void) const{
        return (Eigen::Matrix<Scalar, 7, 1>() << 
            position_, orientation_.coeffs()
        ).finished();
    }

private:
    const Position position_;
    const Orientation orientation_;
};

template<typename Scalar = double>
class RigidBodySecondOrder : public DynamicsVector<PointMassSecondOrder<3, Scalar>, RotationSecondOrder<Scalar>>{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Base = DynamicsVector<PointMassSecondOrder<3, Scalar>, RotationSecondOrder<Scalar>>;
    using PositionBound = BoundBase<3, Scalar>;

    RigidBodySecondOrder(const Scalar& dt, const Scalar& mass, const Vector3& principal_inertia, const Scalar& translational_damping, const Scalar& rotational_damping, const Pose<Scalar>& initial_pose = Pose<Scalar>::Identity())
        :   Base(
                PointMassSecondOrder<3, Scalar>(
                    SecondOrderParameters<Scalar>(
                        dt, mass, translational_damping
                    ), initial_pose.GetPosition()),
                RotationSecondOrder<Scalar>(
                    dt, principal_inertia, rotational_damping,
                    initial_pose.GetOrientation())
            )
    {}

    [[nodiscard]] Pose<Scalar> GetPose(void) const{
        return Pose(Base::GetPosition());
    }

    void SetPositionHardBound(const PositionBound& bound){
        Base::template GetModel<0>().SetHardBound(bound);
    }

    void SetPositionSoftBound(const PositionBound& bound, const Scalar &spring_constant, const Scalar &damping_constant){
        Base::template GetModel<0>().SetSoftBound(bound, spring_constant, damping_constant);
    }
};

}