//----------------------------------------------------------------------------------------------------
// File: MujocoModel.hpp
// Desc: wrapper class for interfacing with MuJoCo
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <string>
#include <array>
#include <mutex>

// Third-party dependencies
#include <mujoco/mujoco.h>

// Project-specific
#include "../DynamicsBase.hpp"

namespace gtfo{

template<unsigned int Dimensions, typename Scalar = double>
class MujocoModel : public DynamicsBase<Dimensions, Scalar>{
public:
    using Base = DynamicsBase<Dimensions, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using MujocoVectorN = Eigen::Matrix<mjtNum, Dimensions, 1>;

    MujocoModel(const std::string& model_file, const Scalar& timestep, const VectorN& initial_position = VectorN::Zero())
        :   Base(initial_position),
            model_(nullptr),
            data_(nullptr)
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
        model_->opt.timestep = timestep;

        data_ = mj_makeData(model_);
        MujocoVectorN::Map(data_->qpos) = Base::position_.template cast<mjtNum>();
    }

    // Copy constructor
    MujocoModel(const MujocoModel& other)
        :   Base(other)
    {
        model_ = mj_copyModel(nullptr, other.model_);
        data_ = mj_copyData(nullptr, other.model_, other.data_);
    }

    // Move constructor
    MujocoModel(MujocoModel&& other) noexcept
        :   Base(other)
    {
        model_ = other.model_;
        data_ = other.data_;

        other.model_ = nullptr;
        other.data_ = nullptr;
    }

    // Assignment operator
    MujocoModel& operator=(const MujocoModel& other){
        // Check against self-assignment
        if(this == &other){
            return *this;
        }

        mj_deleteModel(model_);
        mj_deleteData(data_);
        
        model_ = mj_copyModel(nullptr, other.model_);
        data_ = mj_copyData(nullptr, other.model_, other.data_);  

        return *this;
    }

    // Move assignment operator
    MujocoModel& operator=(MujocoModel&& other) noexcept{
        // Check against self-assignment
        if(this == &other){
            return *this;
        }

        mj_deleteModel(model_);
        mj_deleteData(data_);

        model_ = other.model_;
        data_ = other.data_;

        other.model_ = nullptr;
        other.data_ = nullptr;

        return *this;
    }

    // Destructor
    ~MujocoModel(){
        mj_deleteModel(model_);
        mj_deleteData(data_);
    }

    void SyncSystemTo(const Base& model) override{
        Base::SyncSystemTo(model);
        MujocoVectorN::Map(data_->qpos) = Base::position_.template cast<mjtNum>();
        MujocoVectorN::Map(data_->qvel) = Base::velocity_.template cast<mjtNum>();
        MujocoVectorN::Map(data_->qacc) = Base::acceleration_.template cast<mjtNum>();
    }

    bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        // wrapper_lock_.lock();
        // Update the virtual position if a physical position is given
        const bool err = this->SyncVirtualPositionToPhysical(physical_position);

        // Zero the time-dependent states and do not step if the dynamics are paused
        if(this->DynamicsArePaused()){
            Base::velocity_.setZero();
            Base::acceleration_.setZero();
            MujocoVectorN::Map(data_->qpos).setZero();
            MujocoVectorN::Map(data_->qvel).setZero();
            MujocoVectorN::Map(data_->qacc).setZero();
            return err;
        }

        MujocoVectorN::Map(data_->qpos) = Base::position_.template cast<mjtNum>();

        // Compute all intermediate results dependent on the state, but not the control
        // Note that by using mj_step1 and mj_step2, the integrator must be the default Euler
        // wrapper_lock_.lock();
        mj_step1(model_, data_);
        // wrapper_lock_.unlock();

        // Update the state variables
        Base::position_ = MujocoVectorN::Map(data_->qpos).template cast<Scalar>();
        Base::velocity_ = MujocoVectorN::Map(data_->qvel).template cast<Scalar>();
        const VectorN previous_velocity = MujocoVectorN::Map(data_->qvel).template cast<Scalar>();

        // Apply the control
        MujocoVectorN::Map(data_->ctrl) = (force_input + this->EnforceSoftBound()).template cast<mjtNum>();

        // Finish computing results that depend on the control input
        // wrapper_lock_.lock();
        mj_step2(model_, data_);
        // wrapper_lock_.unlock();

        // Update the state variables after the second step
        Base::position_ = MujocoVectorN::Map(data_->qpos).template cast<Scalar>();
        Base::velocity_ = MujocoVectorN::Map(data_->qvel).template cast<Scalar>();

        // Restrict and set the position and velocity after the integration timestep
        this->EnforceHardBound();
        MujocoVectorN::Map(data_->qpos) = Base::position_.template cast<mjtNum>();
        MujocoVectorN::Map(data_->qvel) = Base::velocity_.template cast<mjtNum>();

        // Update the acceleration with a backward difference
        Base::acceleration_ = (Base::velocity_ - previous_velocity) / model_->opt.timestep;

        // wrapper_lock_.unlock();
        return err;
    }

    mjModel* Get_Model() {
        return model_;
    }

    mjData* Get_Data() {
        return data_;
    }

    // std::mutex Get_Lock() {
    //     return wrapper_lock_;
    // }

private:
    mjModel* model_;
    mjData* data_;
    // std::mutex wrapper_lock_;
};

} // namespace gtfo