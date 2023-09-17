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
    {}

    // PropagateDynamics is empty since each model already contains the relevant dynamics
    void PropagateDynamics(const VectorN& force_input) override{
        return;
    }

    // Step calls the corresponding Step function on each of the models
    void Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{      
        // Any modifications to the input force happen first
        const VectorN modified_force = PremodifyForce(force_input);

        std::apply([&](Models&... models){
            // Index is used to keep track of where each model's dimensions begin in the concatenated VectorN
            size_t index = 0;
            
            // First, step all the models with the appropriate coordinates of the inputs
            ([&]{
                models.Step(
                    modified_force.template block<Models::Dimension, 1>(index, 0),
                    physical_position.template block<Models::Dimension, 1>(index, 0)
                );
                index += Models::Dimension;
            }(), ...);
        }, models_);
    }

    [[nodiscard]] VectorN GetPosition(void) const override{
        VectorN position;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                position.template block<Models::Dimension, 1>(index, 0) = models.GetPosition();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        return position;
    }

    [[nodiscard]] VectorN GetOldPosition(void) const override{
        VectorN old_position;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                old_position.template block<Models::Dimension, 1>(index, 0) = models.GetOldPosition();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        return old_position;
    }

    [[nodiscard]] VectorN GetVelocity(void) const override{
        VectorN velocity;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                velocity.template block<Models::Dimension, 1>(index, 0) = models.GetVelocity();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        return velocity;
    }

    [[nodiscard]] VectorN GetAcceleration(void) const override{
        VectorN acceleration;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                acceleration.template block<Models::Dimension, 1>(index, 0) = models.GetAcceleration();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        return acceleration;
    }

    [[nodiscard]] bool DynamicsArePaused(void) const override{
        bool dynamics_paused = false;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                dynamics_paused &= models.DynamicsArePaused();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        return dynamics_paused;
    }

    [[nodiscard]] VectorN GetSoftBoundRestoringForce(void) const{
        VectorN soft_bound_restoring_force;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                soft_bound_restoring_force.template block<Models::Dimension, 1>(index, 0) = models.GetSoftBoundRestoringForce();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        return soft_bound_restoring_force;
    }

    void SetFullState(const VectorN& position, const VectorN& old_position, const VectorN& velocity, const VectorN& acceleration, const bool& dynamics_paused, const VectorN& soft_bound_restoring_force) override{
        std::apply([&](Models&... models){
            size_t index = 0;
            ([&]{
                models.SetFullState(
                    position.template block<Models::Dimension, 1>(index, 0),
                    old_position.template block<Models::Dimension, 1>(index, 0), 
                    velocity.template block<Models::Dimension, 1>(index, 0),
                    acceleration.template block<Models::Dimension, 1>(index, 0),
                    dynamics_paused,
                    soft_bound_restoring_force.template block<Models::Dimension, 1>(index, 0)
                );
                index += Models::Dimension;
            }(), ...);
        }, models_);
    }

    void SetState(const VectorN& position, const VectorN& velocity, const VectorN& acceleration) override{
        std::apply([&](Models&... models){
            size_t index = 0;
            ([&]{
                models.SetState(
                    position.template block<Models::Dimension, 1>(index, 0), 
                    velocity.template block<Models::Dimension, 1>(index, 0), 
                    acceleration.template block<Models::Dimension, 1>(index, 0)
                );
                index += Models::Dimension;
            }(), ...);
        }, models_);
    }

    void PauseDynamics(const bool& pause) override{
        std::apply([&](Models&... model){
            (model.PauseDynamics(pause), ...);
        }, models_);
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
};

}   // namespace gtfo