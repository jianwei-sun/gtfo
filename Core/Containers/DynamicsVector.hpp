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
#include "../Bounds/BoundBase.hpp"

namespace gtfo{

template <typename... Models>
class DynamicsVector : public DynamicsBase<
    (Models::Dimension + ...), 
    typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType, 
    (Models::PositionDimension + ...)>{
public:
    using Scalar = typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType;
    using Base = DynamicsBase<(Models::Dimension + ...), Scalar, (Models::PositionDimension + ...)>;
    using Bound = BoundBase<(Models::PositionDimension + ...), Scalar>;
    using VectorN = typename Base::VectorN;
    using VectorP = typename Base::VectorP;

    static_assert(std::conjunction_v<std::is_base_of<DynamicsBase<Models::Dimension, Scalar, Models::PositionDimension>, Models>...>, "Models must inherit from DynamicsBase");
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
    void Step(const VectorN& force_input, const VectorP& physical_position = VectorP::Constant(NAN)) override{      
        // Any modifications to the input force happen first
        const VectorN modified_force = Base::PremodifyForce(force_input);

        std::apply([&](Models&... models){
            // Index is used to keep track of where each model's dimensions begin in the concatenated VectorN
            size_t index_n = 0;
            size_t index_p = 0;
            
            // First, step all the models with the appropriate coordinates of the inputs
            ([&]{
                models.Step(
                    modified_force.template block<Models::Dimension, 1>(index_n, 0),
                    physical_position.template block<Models::PositionDimension, 1>(index_p, 0)
                );
                index_n += Models::Dimension;
                index_p += Models::PositionDimension;
            }(), ...);
        }, models_);
    }

    [[nodiscard]] VectorP GetPosition(void) const override{
        VectorP position;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                position.template block<Models::PositionDimension, 1>(index, 0) = models.GetPosition();
                index += Models::PositionDimension;
            }(), ...);
        }, models_);
        return position;
    }

    [[nodiscard]] VectorP GetOldPosition(void) const override{
        VectorP old_position;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                old_position.template block<Models::PositionDimension, 1>(index, 0) = models.GetOldPosition();
                index += Models::PositionDimension;
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
        bool dynamics_paused = true;
        std::apply([&](const Models&... models){
            size_t index = 0;
            ([&]{
                dynamics_paused &= models.DynamicsArePaused();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        return dynamics_paused;
    }

    void SetFullState(const VectorP& position, const VectorP& old_position, const VectorN& velocity, const VectorN& acceleration, const bool& dynamics_paused) override{
        std::apply([&](Models&... models){
            size_t index_n = 0;
            size_t index_p = 0;
            ([&]{
                models.SetFullState(
                    position.template block<Models::PositionDimension, 1>(index_p, 0),
                    old_position.template block<Models::PositionDimension, 1>(index_p, 0), 
                    velocity.template block<Models::Dimension, 1>(index_n, 0),
                    acceleration.template block<Models::Dimension, 1>(index_n, 0),
                    dynamics_paused
                );
                index_n += Models::Dimension;
                index_p += Models::PositionDimension;
            }(), ...);
        }, models_);
    }

    void SetState(const VectorP& position, const VectorN& velocity, const VectorN& acceleration) override{
        std::apply([&](Models&... models){
            size_t index_n = 0;
            size_t index_p = 0;
            ([&]{
                models.SetState(
                    position.template block<Models::PositionDimension, 1>(index_p, 0), 
                    velocity.template block<Models::Dimension, 1>(index_n, 0), 
                    acceleration.template block<Models::Dimension, 1>(index_n, 0)
                );
                index_n += Models::Dimension;
                index_p += Models::PositionDimension;
            }(), ...);
        }, models_);
    }

    void PauseDynamics(const bool& pause) override{
        std::apply([&](Models&... model){
            (model.PauseDynamics(pause), ...);
        }, models_);
    }

    // Bounds are prevented from being set at the container level. They should be set at the submodel level
    void SetHardBound(const Bound& bound) override{
        assert(false);
    }

    void SetSoftBound(const Bound& bound, const Scalar &spring_constant, const Scalar &damping_constant) override{
        assert(false);
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