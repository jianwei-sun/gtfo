//----------------------------------------------------------------------------------------------------
// File: HomingModel.hpp
// Desc: Dynamics model for creating a continuous homing trajectory
//       The homing trajectory is started everytime SyncSystemTo is called, with the initial position
//       given by the position of the argument. 
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "DynamicsBase.hpp"
#include "../Utils/Constants.hpp"

namespace gtfo{

template <unsigned int Dimensions, typename Scalar = double>
class HomingModel : public DynamicsBase<Dimensions, Scalar>{
public:
    using Base = DynamicsBase<Dimensions, Scalar>;
    using VectorN = typename Base::VectorN;

    HomingModel(const Scalar& dt, const Scalar& max_speed, const VectorN& home_position = VectorN::Zero())
        :   Base(home_position),
            dt_(dt),
            time_(0.0),
            duration_(0.0),
            starting_position_(home_position),
            first_bend_position_(home_position),
            second_bend_position_(home_position),
            home_position_(home_position),
            direction_(VectorN::Zero()),
            max_speed_(max_speed)
    {
        assert(max_speed_ > 0.0);
    }

    // ResetHoming is called to compute a new homing trajectory from the passed-in starting_position
    // to home_position, such that the trajectory moves at max_speed_ for 80% of the time
    void ResetHoming(const VectorN& starting_position){
        // Restart the timer and compute the overall duration based on max_speed
        time_ = 0.0;
        duration_ = (home_position_ - starting_position).norm() * (2.0 / 1.8) / max_speed_;
        direction_ = (home_position_ - starting_position).normalized();

        // Compute the key positions in the homing trajectory
        starting_position_ = starting_position;
        first_bend_position_ = starting_position_ + (0.05 * max_speed_ * duration_) * direction_;
        second_bend_position_ = first_bend_position_ + (max_speed_ * 0.8 * duration_) * direction_;

        // Set the state to the starting state
        Base::old_position_ = Base::position_;
        Base::position_ = starting_position_;
        Base::velocity_.setZero();
        Base::acceleration_.setZero();
    }

    // If HomingModel is used in DynamicsSelector, then homing is automatically reset when the model
    // is selected
    void SyncSystemTo(const Base& model) override{
        ResetHoming(model.GetPosition());
        Base::PauseDynamics(model.DynamicsArePaused());
    }

    void PauseDynamics(const bool& pause) override{
        if(DynamicsArePaused() && !pause){
            ResetHoming(Base::position_);
        }
        Base::PauseDynamics(pause);
    }

    void PropagateDynamics(const VectorN& force_input) override{
        // Time update
        time_ += dt_;

        // If duration_ is invalidly small, or homing hasn't been reset yet, then do not move
        if(duration_ < GTFO_EQUALITY_COMPARISON_TOLERANCE){
            return;
        }

        // In first 10% of homing
        if(time_ < 0.1 * duration_){
            const Scalar acceleration = max_speed_ / (0.1 * duration_);
            const Scalar speed = acceleration * time_;

            Base::position_ = starting_position_ + (0.5 * speed * time_) * direction_;
            Base::velocity_ = speed * direction_;
            Base::acceleration_ = acceleration * direction_;
        } 
        // 10% - 90% of homing
        else if(0.1 * duration_ <= time_ && time_ < 0.9 * duration_){
            const Scalar segment_time = time_ - 0.1 * duration_;

            Base::position_ = first_bend_position_ + (segment_time * max_speed_) * direction_;
            Base::velocity_ = max_speed_ * direction_;
            Base::acceleration_.setZero();
        } 
        // 90% - 100% of homing
        else if(0.9 * duration_ <= time_ && time_ < duration_){
            const Scalar segment_time = time_ - 0.9 * duration_;
            const Scalar acceleration = -max_speed_ / (0.1 * duration_);
            const Scalar speed = max_speed_ + acceleration * segment_time;
            const Scalar distance = (max_speed_ + 0.5 * acceleration * segment_time) * segment_time;

            Base::position_ = second_bend_position_ + distance * direction_;
            Base::velocity_ = speed * direction_;
            Base::acceleration_ = acceleration * direction_;
        } 
        // After homing
        else{ 
            Base::position_ = home_position_;
            Base::velocity_.setZero();
            Base::acceleration_.setZero();
        }
    }

    Scalar GetProgress(void) const{
        if(duration_ < GTFO_EQUALITY_COMPARISON_TOLERANCE){
            return static_cast<Scalar>(time_ >= duration_);
        } else{
            return time_ / duration_;
        }
    }

private:
    // Time-related
    const Scalar dt_;
    Scalar time_;
    Scalar duration_;

    // Position-related
    VectorN starting_position_;
    VectorN first_bend_position_;
    VectorN second_bend_position_;
    const VectorN home_position_;

    // Velocity-related
    VectorN direction_;
    const Scalar max_speed_;
};

}   // namespace gtfo