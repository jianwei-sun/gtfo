//----------------------------------------------------------------------------------------------------
// File: DynamicsVector.hpp
// Desc: reference class, which serves as a container for models.
//       This container allows combining several different models (all inheriting from DynamicsBase)
//       into a single object to simplify situations in which all models need to be accessed. The 
//       container is treated as a dynamics model, where its state is the concatenated states of its
//       models. Its state can stepped with DynamicsVector::Step, which accepts vectors whose 
//       dimensions equal the sum of all the model dimensions.
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <type_traits>
#include <memory>
#include <tuple>

// Project-specific
#include "../Models/DynamicsBase.hpp"

namespace gtfo{

template <typename... Models>
class DynamicsVector : public DynamicsBase<(Models::Dimension + ...), typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType>{
public:
    using Scalar = typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType;
    using Base = DynamicsBase<(Models::Dimension + ...), Scalar>;
    using VectorN = typename Base::VectorN;

    static_assert(std::conjunction_v<std::is_base_of<DynamicsBase<Models::Dimension, Scalar>, Models>...>, "Models must inherit from DynamicsBase");
    static_assert(std::conjunction_v<std::is_same<Scalar, typename Models::ScalarType>...>, "Models must have the same Scalar type");

    // Constructs a DynamicsVector with the models passed in as arguments. Each model must inherit from DynamicsBase. 
    DynamicsVector(const Models&... models)
        :   Base(),
            models_(models...)
    {    
        // Update the state variables of DynamicsVector. Use an index to keep track of where each model's dimensions begin
        size_t index = 0;
        ([&]{
            Base::position_.template block<Models::Dimension, 1>(index, 0) = models.GetPosition();
            Base::old_position_.template block<Models::Dimension, 1>(index, 0) = models.GetOldPosition();
            Base::velocity_.template block<Models::Dimension, 1>(index, 0) = models.GetVelocity();
            Base::acceleration_.template block<Models::Dimension, 1>(index, 0) = models.GetAcceleration();
            Base::soft_bound_restoring_force_.template block<Models::Dimension, 1>(index, 0) = models.GetSoftBoundRestoringForce();
            Base::dynamics_paused_ |= models.DynamicsArePaused();
            index += Models::Dimension;
        }(), ...);
    }

    // PropagateDynamics is empty since each model already contains the relevant dynamics
    void PropagateDynamics(const VectorN& force_input) override{
        return;
    }

    // Step calls the corresponding Step function on each of the models
    bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{      
        // Any modifications to the input force happen first
        const VectorN modified_force = PremodifyForce(force_input);

        bool result = true;
        std::apply([&](Models&... models){
            // Index is used to keep track of where each model's dimensions begin in the concatenated VectorN
            size_t index = 0;
            
            // First, step all the models with the appropriate coordinates of the inputs
            ([&]{
                result &= models.Step(
                    modified_force.template block<Models::Dimension, 1>(index, 0),
                    physical_position.template block<Models::Dimension, 1>(index, 0)
                );
                index += Models::Dimension;
            }(), ...);
        }, models_);

        UpdateVectorState();

        return result;
    }

    // Applies a lambda that accepts a model and index to each model in the tuple.
    // For example: [&](auto& model, const size_t& i){...}
    template <typename Lambda>
    void Apply(const Lambda& lambda){
        std::apply([&](Models&... models){
            size_t model_index = 0;
            ([&]{
                lambda(models, model_index);
                model_index++;
            }(), ...);
        }, models_);
    }

    void PauseDynamics(const bool& pause) override{
        std::apply([&](Models&... model){
            (model.PauseDynamics(pause), ...);
        }, models_);
        Base::PauseDynamics(pause);
    }

    void SyncSystemTo(const Base& model) override{
        std::apply([&](Models&... models){
            size_t index = 0;
            ([&]{
                models.SetOldPosition(model.GetOldPosition().template block<Models::Dimension, 1>(index, 0));
                models.SetAcceleration(model.GetAcceleration().template block<Models::Dimension, 1>(index, 0));
                models.SetSoftBoundRestoringForce(model.GetSoftBoundRestoringForce().template block<Models::Dimension, 1>(index, 0));
                models.SetPositionAndVelocity(
                    model.GetPosition().template block<Models::Dimension, 1>(index, 0),
                    model.GetVelocity().template block<Models::Dimension, 1>(index, 0),
                    false
                );
                models.PauseDynamics(model.DynamicsArePaused());

                index += Models::Dimension;
            }(), ...);
        }, models_);

        UpdateVectorState();
    }

    void SetPositionAndVelocity(const VectorN& position, const VectorN& velocity, const bool& bypass_checks = false) override{
        std::apply([&](Models&... models){
            size_t index = 0;
            ([&]{
                models.SetPositionAndVelocity(
                    position.template block<Models::Dimension, 1>(index, 0),
                    velocity.template block<Models::Dimension, 1>(index, 0), 
                    bypass_checks);
                index += Models::Dimension;
            }(), ...);
        }, models_);

        UpdateVectorState();
    }

    template <size_t index>
    typename std::tuple_element<index, std::tuple<Models...>>::type& GetModel(){
        return std::get<index>(models_);
    }

    template <size_t index>
    const typename std::tuple_element<index, std::tuple<Models...>>::type& GetModel() const{
        return std::get<index>(models_);
    }

private:
    std::tuple<Models...> models_;

    void UpdateVectorState(){
        std::apply([&](Models&... models){
            size_t index = 0;
            ([&]{
                Base::position_.template block<Models::Dimension, 1>(index, 0) = models.GetPosition();
                Base::old_position_.template block<Models::Dimension, 1>(index, 0) = models.GetOldPosition();
                Base::velocity_.template block<Models::Dimension, 1>(index, 0) = models.GetVelocity();
                Base::acceleration_.template block<Models::Dimension, 1>(index, 0) = models.GetAcceleration();
                Base::soft_bound_restoring_force_.template block<Models::Dimension, 1>(index, 0) = models.GetSoftBoundRestoringForce();
                index += Models::Dimension;
            }(), ...);
        }, models_);

        // All the models should have the same pause state
        Base::dynamics_paused_ = std::get<0>(models_).DynamicsArePaused();
    }
};

}   // namespace gtfo