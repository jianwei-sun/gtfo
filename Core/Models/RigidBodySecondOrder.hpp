//----------------------------------------------------------------------------------------------------
// File: RigidBodySecondOrder.hpp
// Desc: Full rigid body dynamics
//----------------------------------------------------------------------------------------------------
#pragma once

#include <cassert>
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
// RigidBodySecondOrder(const Scalar& dt, const Scalar& mass, const Vector3& principal_inertia, const Scalar& translational_damping, const Scalar& rotational_damping, const Pose<Scalar>& initial_pose = Pose<Scalar>::Identity())
    // ---------------------------Now accepts arguments for stiffness and virtual_spring_zero_position----------------------------------------------
    // ----------------------------------------Update 11/5/25 by RICK REN --------------------------------------------------------------------------
    RigidBodySecondOrder(const Scalar& dt, const Scalar& mass, const Vector3& principal_inertia, const Scalar& translational_damping, const Scalar& rotational_damping, const Pose<Scalar>& initial_pose = Pose<Scalar>::Identity(), const Scalar& stiffness=Scalar(0), const Scalar& virtual_spring_zero_position=Scalar(0))
        :   Base(
                PointMassSecondOrder<3, Scalar>(
                    SecondOrderParameters<Scalar>(
                        dt, mass, translational_damping, stiffness, virtual_spring_zero_position
                    ), initial_pose.GetPosition()),
                RotationSecondOrder<Scalar>(
                    dt, principal_inertia, rotational_damping,
                    initial_pose.GetOrientation())
            )
    {}

    // NEW overload: same params, but last arg is a Vector3 x0 <- added 11/5/25
    // RigidBodySecondOrder(const Scalar& dt, const Scalar& mass, const Vector3& principal_inertia,
    //                     const Scalar& translational_damping, const Scalar& rotational_damping,
    //                     const Pose<Scalar>& initial_pose,
    //                     const Scalar& stiffness,
    //                     const Vector3& x0_vec)
    // : Base(
    //     PointMassSecondOrder<3, Scalar>(
    //         // pass scalar x0=0.0 here; we’ll set the real vector below
    //         SecondOrderParameters<Scalar>(dt, mass, translational_damping, stiffness, Scalar(0)),
    //         initial_pose.GetPosition()),
    //     RotationSecondOrder<Scalar>(
    //         dt, principal_inertia, rotational_damping,
    //         initial_pose.GetOrientation())
    // )
    // {
    //     // Immediately overwrite scalar x0 with the per-axis vector
    //     Base::template GetModel<0>().SetVirtualSpringZero(x0_vec);
    // }
    RigidBodySecondOrder(const Scalar& dt,
                     const Scalar& mass,
                     const Vector3& principal_inertia,
                     const Scalar& translational_damping,
                     const Scalar& rotational_damping,
                     const Pose<Scalar>& initial_pose,
                     const Scalar& stiffness,
                     const Eigen::Matrix<Scalar,3,1>& x0_vec)
    : Base(
        // note: calling the new ctor that takes x0_vec directly
        PointMassSecondOrder<3, Scalar>(
            SecondOrderParameters<Scalar>(dt, mass, translational_damping, stiffness, Scalar(0)), // scalar x0 unused
            initial_pose.GetPosition(),
            x0_vec
        ),
        RotationSecondOrder<Scalar>(
            dt, principal_inertia, rotational_damping,
            initial_pose.GetOrientation()
        )
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

    // --------------------Update by RICK 08/16/25----------------------------
    // update translational damping
    inline void SetTranslationalDamping(Scalar d){
        Base::template GetModel<0>().SetDamping(d);
    }

    Scalar GetTranslationalDamping() const{

        return Base::template GetModel<0>().GetDamping();
    }
    //update rotational damping
    inline void SetRotationalDamping(Scalar d){
        Base::template GetModel<1>().SetDamping(d);
    }
    Scalar GetRotationalDamping() const{

        return Base::template GetModel<1>().GetDamping();
    }
    // --------------------Update by RICK 11/04/25-----------------------
    inline void SetTranslationalStiffness(Scalar k){
        Base::template GetModel<0>().SetStiffness(k);
    }
    inline Scalar GetTranslationalStiffness() const{
        return Base::template GetModel<0>().GetStiffness();
    }
    // inline void SetTranslationalSpringZero(Scalar x0){
    //     Base::template GetModel<0>().SetVirtualSpringZero(x0);
    // }
    // inline Scalar GetTranslationalSpringZero() const{
    //     return Base::template GetModel<0>().GetVirtualSpringZero();
    // }

    // // RENAME the getters to avoid signature collision
    // inline Scalar GetTranslationalSpringZeroScalar() const {
    //     return Base::template GetModel<0>().GetVirtualSpringZero();
    // }
    // inline Vector3 GetTranslationalSpringZeroVec() const {
    //     return Base::template GetModel<0>().GetVirtualSpringZeroVec();
    // }
    // inline void SetTranslationalSpringZero(const Vector3& x0_vec){
    //     Base::template GetModel<0>().SetVirtualSpringZero(x0_vec);
    // }
};

}