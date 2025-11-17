//----------------------------------------------------------------------------------------------------
// File: PointMassSecondOrder.hpp
// Desc: a second-order dynamics model
//----------------------------------------------------------------------------------------------------
#pragma once
// Project-specific
#include "PointMassBase.hpp"
#include <cassert>

namespace gtfo{

    template <typename Scalar = double>
    struct SecondOrderParameters : ParametersBase<Scalar>
    {
        Scalar mass;
        Scalar damping;
        Scalar stiffness;
        Scalar virtual_spring_zero_position;

        SecondOrderParameters()
            : ParametersBase<Scalar>(), mass(1.0), damping(1.0), stiffness(0.0), virtual_spring_zero_position(0.0)
        {
        }

        // overloaded constructor
        SecondOrderParameters(const Scalar& dt, const Scalar& mass, const Scalar& damping)
            : ParametersBase<Scalar>(dt), mass(mass), damping(damping), stiffness(0.0), virtual_spring_zero_position(0.0)
        {
            assert(mass > 0.0 && damping > 0.0);
        }

        SecondOrderParameters(const Scalar& dt, const Scalar& mass, const Scalar& damping, const Scalar& stiffness, const Scalar& virtual_spring_zero_position)
            : ParametersBase<Scalar>(dt), mass(mass), damping(damping), stiffness(stiffness), virtual_spring_zero_position(virtual_spring_zero_position)
        {
            assert(mass > 0.0 && damping > 0.0);
        }

        SecondOrderParameters operator+(const SecondOrderParameters& other){
            return SecondOrderParameters(ParametersBase<Scalar>::dt, 
                mass + other.mass, 
                damping + other.damping,
                stiffness + other.stiffness,
                virtual_spring_zero_position + other.virtual_spring_zero_position);
        }

        SecondOrderParameters operator*(const Scalar& scalar){
            return SecondOrderParameters(ParametersBase<Scalar>::dt, 
                scalar * mass, 
                scalar * damping,
                scalar * stiffness,
                scalar * virtual_spring_zero_position);
        }
    };

    template <unsigned int Dimensions, typename Scalar = double>
    class PointMassSecondOrder : public PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>
    {
    public:
        using Base = PointMassBase<Dimensions, SecondOrderParameters<Scalar>, Scalar>;
        using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

        PointMassSecondOrder(const SecondOrderParameters<Scalar> &parameters, const VectorN &initial_position = VectorN::Zero())
            : Base(parameters, initial_position) //, spring_zero_vec_(VectorN::Constant(parameters.virtual_spring_zero_position)) // <- added 11/5/25
        {
            SetStateTransitionMatrices(parameters);
        }

        // // overloaded constructor for vector spring zero position 11/04/25
        // PointMassSecondOrder(const SecondOrderParameters<Scalar>& parameters,
        //              const VectorN& initial_position,
        //              const VectorN& x0_vec)
        //     : Base(parameters, initial_position), spring_zero_vec_(x0_vec)
        // {
        //     SetStateTransitionMatrices(parameters);
        // }

        // --------------------Update by RICK 08/16/25----------------------------
        // added for variable damping
        void SetDamping(Scalar d_new){
            Base::parameters_.damping = std::max<Scalar>(0, d_new);
            SetStateTransitionMatrices(Base::parameters_);
        }
        Scalar GetDamping() const{

            return Base::parameters_.damping;
        }
        // ---------------------Update by RICK 11/04/25-----------------------------
        void SetStiffness(Scalar k_new){
            Base::parameters_.stiffness = std::max<Scalar>(0, k_new);
            SetStateTransitionMatrices(Base::parameters_);
        }
        Scalar GetStiffness() const { 
            return Base::parameters_.stiffness; 
        }

        // ---------------------Update by RICK 11/13/25-----------------------------
        void SetVirtualSpringZero(Scalar x0){
            Base::parameters_.virtual_spring_zero_position = x0;
            SetStateTransitionMatrices(Base::parameters_);
        }
        Scalar GetVirtualSpringZero() const {
            return Base::parameters_.virtual_spring_zero_position;
        }

        // void SetVirtualSpringZero(Scalar x0_scalar){
        //     spring_zero_vec_.setConstant(x0_scalar);
        //     SetStateTransitionMatrices(Base::parameters_);
        // }
        // Scalar GetVirtualSpringZero() const { return spring_zero_vec_.mean(); }

        // // NEW: vector API for (x0x, x0y, x0z) <- 11/5/25
        // void SetVirtualSpringZero(const VectorN& x0_vec){
        //     spring_zero_vec_ = x0_vec;
        //     SetStateTransitionMatrices(Base::parameters_);
        // }
        // VectorN GetVirtualSpringZeroVec() const { return spring_zero_vec_; }


        // ------------------------------------------------------------------------
        // Propagate dynamics for a second order system but using softbounds if they exist
        void PropagateDynamics(const VectorN &force_input) override
        {
            Base::PropagateDynamics(force_input);
            
            // Calculate the acceleration using the more accurate continuous equations with the current velocity
            Base::acceleration_ = (-Base::parameters_.damping / Base::parameters_.mass) * Base::velocity_ + (-Base::parameters_.stiffness 
            / Base:: parameters_.mass) * (Base::position_ - VectorN::Constant(Base::parameters_.virtual_spring_zero_position)) + force_input / Base::parameters_.mass;
        }

        // // added 11/5/25
        // void PropagateDynamics(const VectorN& F_ext) override {
        //     const Scalar k = Base::parameters_.stiffness;
        //     const VectorN F_spring = (-k) * (Base::position_ - spring_zero_vec_); // per-axis
        //     const VectorN F_total  = F_ext + F_spring;

        //     Base::PropagateDynamics(F_total);  // state sees x0 in all axes

        //     // optional: consistent accel for logging
        //     const Scalar m = Base::parameters_.mass;
        //     const Scalar d = Base::parameters_.damping;
        //     Base::acceleration_ = (-d/m) * Base::velocity_ + F_total / m;
        // }



    private:
        void SetStateTransitionMatrices(const SecondOrderParameters<Scalar> &parameters) override
        {
            const Scalar& dt = parameters.dt;
            const Scalar& mass = parameters.mass;
            const Scalar& damping = parameters.damping;
            const Scalar& stiffness = parameters.stiffness;
            const Scalar& virtual_spring_zero_position = parameters.virtual_spring_zero_position;

            // Update the discrete-time state transition matrices, which are computed using exact discretization
            if (stiffness == 0){
                const Scalar exponent = std::exp(-damping / mass * dt);
                Base::A_discrete_ << static_cast<Scalar>(1.0), (static_cast<Scalar>(1.0) - exponent) * mass / damping,
                    static_cast<Scalar>(0.0), exponent;
                Base::B_discrete_ << (damping * dt - (static_cast<Scalar>(1.0) - exponent) * mass) / (damping * damping),
                    (static_cast<Scalar>(1.0) - exponent) / damping;}
            else {
            // here we used a first order approximation
                Base::A_discrete_ << static_cast<Scalar>(1.0), dt,
                    - (dt * stiffness) / mass, static_cast<Scalar>(1.0) - (damping * dt) / mass;
                Base::B_discrete_ << static_cast<Scalar>(0.0),
                    dt/mass;
                }
            Base::C_discrete_ << static_cast<Scalar>(0.0), (stiffness * dt * virtual_spring_zero_position)/ mass; // the affine term
        }

        // // added 11/5/25
        // void SetStateTransitionMatrices(const SecondOrderParameters<Scalar>& p) override {
        //     const Scalar dt = p.dt, m = p.mass, d = p.damping;
        //     const Scalar e  = std::exp(-d/m * dt);
        //     Base::A_discrete_ << Scalar(1), (Scalar(1)-e)*m/d,
        //                         Scalar(0), e;
        //     Base::B_discrete_ << (d*dt - (Scalar(1)-e)*m)/(d*d),
        //                         (Scalar(1)-e)/d;
        //     Base::C_discrete_.setZero();     // no affine bias
        // }

        // VectorN spring_zero_vec_;   // <- NEW: per-axis (x0x, x0y, x0z)

        
    };

} // namespace gtfo
