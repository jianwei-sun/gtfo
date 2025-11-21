//--------------------------------------------------------------------------------------------------
// File: RigidBody6DoFSeparated.hpp
// Desc: 6-DoF virtual admittance composed of:
//       - Translation: 3 independent PointMassSecondOrder<1>
//       - Rotation:    RotationSecondOrder<Scalar> (quaternion-based)
//--------------------------------------------------------------------------------------------------
#pragma once

#include <Eigen/Dense>
#include "PointMassSecondOrder.hpp"    // gtfo::PointMassSecondOrder, gtfo::SecondOrderParameters
#include "RotationSecondOrder.hpp"     // gtfo::RotationSecondOrder
#include "../Bounds/RectangleBound.hpp"          // gtfo::RectangleBound

namespace gtfo {

template<typename Scalar = double>
class RigidBody6DoFSeparated {
public:
  using Vec1   = Eigen::Matrix<Scalar,1,1>;
  using Vec3   = Eigen::Matrix<Scalar,3,1>;
  using Quat   = Eigen::Quaternion<Scalar>;

  using PM1D     = PointMassSecondOrder<1, Scalar>;
  using RotSO    = RotationSecondOrder<Scalar>;
  using Bound1D  = RectangleBound<1, Scalar>;

  using PoseT = gtfo::Pose<Scalar>;

  struct TransParams {
    Scalar m[3]   {Scalar(1), Scalar(1), Scalar(1)};
    Scalar d[3]   {Scalar(1), Scalar(1), Scalar(1)};
    Scalar k[3]   {Scalar(0), Scalar(0), Scalar(0)};
    Scalar x0[3]  {Scalar(0), Scalar(0), Scalar(0)};
  };

  struct RotParams {
    Vec3   I      {Scalar(1), Scalar(1), Scalar(1)}; // principal inertia
    Scalar d      {Scalar(1)};                       // rotational damping
  };

  // Constructor
  RigidBody6DoFSeparated(Scalar dt,
                         const TransParams& T,
                         const RotParams&  R,
                         const PoseT& initial_pose = PoseT::Identity())
  : x_( SecondOrderParameters<Scalar>(dt, T.m[0], T.d[0], T.k[0], T.x0[0]),
        Vec1::Constant(initial_pose.GetPosition().x()) ),
    y_( SecondOrderParameters<Scalar>(dt, T.m[1], T.d[1], T.k[1], T.x0[1]),
        Vec1::Constant(initial_pose.GetPosition().y()) ),
    z_( SecondOrderParameters<Scalar>(dt, T.m[2], T.d[2], T.k[2], T.x0[2]),
        Vec1::Constant(initial_pose.GetPosition().z()) ),
    rot_( dt, R.I, R.d, initial_pose.GetOrientation() )
  {}

  // Steping dynamics
  inline void Step(const Vec3& F, const Vec3& Tau) {
    x_.Step(Vec1::Constant(F.x()));
    y_.Step(Vec1::Constant(F.y()));
    z_.Step(Vec1::Constant(F.z()));
    rot_.Step(Tau); // RotationSecondOrder takes Vec3 torque
  }

  inline void PauseDynamics(bool flag)
  {
    x_.PauseDynamics(flag);
    y_.PauseDynamics(flag);
    z_.PauseDynamics(flag);
    rot_.PauseDynamics(flag);
  }

  // --- Translational getters---
  inline Vec3 GetPosition() const {
    return Vec3(x_.GetPosition()(0), y_.GetPosition()(0), z_.GetPosition()(0));
  }
  inline Vec3 GetLinearVelocity() const {
    return Vec3(x_.GetVelocity()(0), y_.GetVelocity()(0), z_.GetVelocity()(0));
  }

  // --- Rotational getters ---
  inline Quat GetOrientation() const { return rot_.GetOrientation(); }
  inline Vec3 GetAngularVelocity() const { return rot_.GetVelocity(); }

  inline PoseT GetPose() const {
    return PoseT(GetPosition(), GetOrientation());
  }

  inline Eigen::Matrix<Scalar,6,1> GetVelocityFull() const {
    Eigen::Matrix<Scalar,6,1> vel;
    vel.template head<3>() = GetLinearVelocity();
    vel.template tail<3>() = GetAngularVelocity();
    return vel;
  }
  // --- Per-axis linear---
  inline Vec3 GetTranslationalDamping() const {return Vec3(x_.GetDamping(), y_.GetDamping(), z_.GetDamping());}
  inline Vec3 GetStiffness() const {return Vec3(x_.GetStiffness(), y_.GetStiffness(), z_.GetStiffness());}
  inline void SetTranslationalDamping (const Vec3& d){ x_.SetDamping(d.x()); y_.SetDamping(d.y()); z_.SetDamping(d.z()); }
  inline void SetTranslationalStiffness(const Vec3& k){ x_.SetStiffness(k.x()); y_.SetStiffness(k.y()); z_.SetStiffness(k.z()); }
  inline void SetSpringZero (const Vec3& x0){ x_.SetVirtualSpringZero(x0.x()); y_.SetVirtualSpringZero(x0.y()); z_.SetVirtualSpringZero(x0.z()); }

  // --------Rotational-------
  inline void SetRotationalDamping(Scalar d){ rot_.SetDamping(d); }

  // ------- Rectangular bound per-axis------
  inline void SetRectangularHardBound(const Vec3& lo_abs, const Vec3& hi_abs, const Vec3& center_abs){
    // Use Vec1 (which depends on Scalar) to avoid hard-coded float matrix types
    Bound1D bound_x(Vec1::Constant(center_abs[0] + lo_abs[0]),
                    Vec1::Constant(center_abs[0] + hi_abs[0]),
                    Vec1::Constant(center_abs[0]));
    Bound1D bound_y(Vec1::Constant(center_abs[1] + lo_abs[1]),
                    Vec1::Constant(center_abs[1] + hi_abs[1]),
                    Vec1::Constant(center_abs[1]));

    Bound1D bound_z(Vec1::Constant(center_abs[2] + lo_abs[2]),
                    Vec1::Constant(center_abs[2] + hi_abs[2]),
                    Vec1::Constant(center_abs[2]));

    x_.SetHardBound(bound_x);
    y_.SetHardBound(bound_y);
    z_.SetHardBound(bound_z);
  }

  inline void SetRectangularSoftBound(const Vec3& lo_abs, const Vec3& hi_abs,
                                  const Vec3& center_abs,
                                  const Vec3& k_wall, const Vec3& c_wall){
    x_.SetSoftBound(Make1DBound(lo_abs.x(), hi_abs.x(), center_abs.x()), k_wall.x(), c_wall.x());
    y_.SetSoftBound(Make1DBound(lo_abs.y(), hi_abs.y(), center_abs.y()), k_wall.y(), c_wall.y());
    z_.SetSoftBound(Make1DBound(lo_abs.z(), hi_abs.z(), center_abs.z()), k_wall.z(), c_wall.z());
  }

  inline void SetVelocityLimit(Vec3 max_vel)
  {
    x_.SetVelocityLimit(max_vel[0]);
    y_.SetVelocityLimit(max_vel[1]);
    z_.SetVelocityLimit(max_vel[2]);
  }

  // Direct access if you need to call lower-level APIs
  inline PM1D& X() { return x_; }     inline const PM1D& X() const { return x_; }
  inline PM1D& Y() { return y_; }     inline const PM1D& Y() const { return y_; }
  inline PM1D& Z() { return z_; }     inline const PM1D& Z() const { return z_; }
  inline RotSO& Rot() { return rot_; }inline const RotSO& Rot() const { return rot_; }

private:
  static inline Bound1D Make1DBound(Scalar lo_abs, Scalar hi_abs, Scalar center_abs){
    // RectangleBound<1> expects lower/upper *relative to* center + center
    return Bound1D( Vec1::Constant(lo_abs - center_abs),
                    Vec1::Constant(hi_abs - center_abs),
                    Vec1::Constant(center_abs) );
  }

  PM1D  x_, y_, z_;
  RotSO rot_;
};

} // namespace gtfo
