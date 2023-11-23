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
    using Pose = Pose<Scalar>;

    RigidBodySecondOrder(const Scalar& dt, const Scalar& mass, const Vector3& principal_inertia, const Scalar& translational_damping, const Scalar& rotational_damping, const Pose& initial_pose = Pose::Identity())
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

    [[nodiscard]] Pose GetPose(void) const{
        return Pose(GetPosition(), GetOrientation());
    }

    [[nodiscard]] Vector3 GetPosition(void) const{
        return Base::template GetModel<0>().GetPosition();
    }

    [[nodiscard]] Vector3 GetOrientation(void) const{
        return Base::template GetModel<1>().GetOrientation();
    }

    void SetPositionHardBound(const PositionBound& bound){
        Base::template GetModel<0>().SetHardBound(bound);
    }

    void SetPositionSoftBound(const PositionBound& bound, const Scalar &spring_constant, const Scalar &damping_constant){
        Base::template GetModel<0>().SetSoftBound(bound, spring_constant, damping_constant);
    }
};

}