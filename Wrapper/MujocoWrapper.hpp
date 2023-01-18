//----------------------------------------------------------------------------------------------------
// File: MujocoWrapper.hpp
// Desc: wrapper class for interfacing with MuJoCo
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <string>
#include <array>

// Third-party dependencies
#include <mujoco/mujoco.h>

// Project-specific
#include "../Core/Models/DynamicsBase.hpp"

namespace gtfo{

template<unsigned int Dimensions>
class MujocoWrapper : public DynamicsBase<Dimensions, mjtNum>{
public:
    using Base = DynamicsBase<Dimensions, mjtNum>;
    using VectorN = Eigen::Matrix<mjtNum, Dimensions, 1>;

    MujocoWrapper(const std::string& model_file, const mjtNum& timestep)
        :   acceleration_(VectorN::Zero()),
            model_(nullptr),
            data_(nullptr),
            timestep_(timestep)
    {
        // Ensure header and compiled library versions match
        assert(mjVERSION_HEADER == mj_version());

        assert(timestep > 0.0);

        // Parse and compile the model from file
        // TODO: update this to load a precompiled MJB file, which gets generated by cmake
        std::array<char, 1000> error;
        model_ = mj_loadXML(model_file.c_str(), NULL, error.data(), 1000);
        if(!model_){
            mju_error("Could not load model from file");
        }

        // Ensure the dimensions of the model state and control match up
        if(Dimensions != model_->nv){
            mju_error("Dimensions of Wrapper and MuJoCo model state are mismatched");
        }
        if(model_->nv != model_->nu){
            mju_error("Dimensions of MuJoCo model state and control are mismatched");
        }

        // Set simultation parameters
        model_->opt.timestep = timestep_;

        data_ = mj_makeData(model_);
    }

    void Step(const VectorN& user_input, const VectorN& environment_input) override{
        // Compute all intermediate results dependent on the state, but not the control
        // Note that by using mj_step1 and mj_step2, the integrator must be the default Euler
        mj_step1(model_, data_);

        // Update the state variables
        Base::position_ = VectorN::Map(data_->qpos);
        Base::velocity_ = VectorN::Map(data_->qvel);
        const VectorN previous_velocity = VectorN::Map(data_->qvel);

        // Apply the control
        VectorN::Map(data_->ctrl) = user_input + environment_input + this->EnforceSoftBound();

        // Finish computing results that depend on the control input
        mj_step2(model_, data_);

        // Restrict and set the position and velocity after the integration timestep
        this->EnforceHardBound();
        VectorN::Map(data_->qpos) = Base::position_;
        VectorN::Map(data_->qvel) = Base::velocity_;

        // Update the acceleration with a backward difference
        acceleration_ = (Base::velocity_ - previous_velocity) / timestep_;
    }

    ~MujocoWrapper(){
        mj_deleteModel(model_);
        mj_deleteData(data_);
    }

private:
    VectorN acceleration_;

    mjModel* model_;
    mjData* data_;

    const mjtNum timestep_;

};

} // namespace gtfo