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
            Base::velocity_.template block<Models::Dimension, 1>(index, 0) = models.GetVelocity();
            Base::acceleration_.template block<Models::Dimension, 1>(index, 0) = models.GetAcceleration();
            index += Models::Dimension;
        }(), ...);
    }

    // Step allows stepping all the models in DynamicsVector as if the container is a single model. Step only
    // returns true if all models' Step functions return true
    bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        bool result = true;

        std::apply([&](Models&... model){
            // Index is used to keep track of where each model's dimensions begin in the concatenated VectorN
            size_t index = 0;

            // First, step all the models with the appropriate coordinates of the inputs
            ([&]{
                result &= model.Step(force_input.template block<Models::Dimension, 1>(index, 0), physical_position.template block<Models::Dimension, 1>(index, 0));
                index += Models::Dimension;
            }(), ...);

            // Then, update the state variables of DynamicsVector accordingly
            index = 0;
            ([&]{
                Base::position_.template block<Models::Dimension, 1>(index, 0) = model.GetPosition();
                Base::velocity_.template block<Models::Dimension, 1>(index, 0) = model.GetVelocity();
                Base::acceleration_.template block<Models::Dimension, 1>(index, 0) = model.GetAcceleration();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        
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