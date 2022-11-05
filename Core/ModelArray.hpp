//----------------------------------------------------------------------------------------------------
// File: ModelArray.hpp
// Desc: a virtual system consisting of a model with dynamics
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <tuple>

// Project-specific
#include "Models/PointMassBase.hpp"

namespace gtfo{

template<typename ...Models>
class ModelArray : private std::tuple<Models...>{
public:
    ModelArray(){}

    template<size_t Index, typename State> 
    void GetState(State& state) const {
        const auto& model = std::get<Index>(*this);

        state.position = model.GetPosition();
        state.velocity = model.GetVelocity();
        state.acceleration = model.GetAcceleration();
    }

    template<size_t Index, typename InputType>
    void Step(const InputType& input){
        std::get<Index>(*this).Step(input);
    }

    template<size_t Index, typename Parameter>
    void UpdateParameters(const Parameter& parameter){
        std::get<Index>(*this).SetParameters(parameter);
    }

    template<size_t I = 0, typename ...States>
    void GetAllStates(States&... states) const {
        auto& state = std::get<I>(std::forward_as_tuple(states...));
        const auto& model = std::get<I>(*this);

        state.position = model.GetPosition();
        state.velocity = model.GetVelocity();
        state.acceleration = model.GetAcceleration();

        if constexpr(I + 1 != sizeof...(States)){
            GetAllStates<I + 1>(states...);
        }
    }

    template<size_t I = 0, typename ...InputTypes>
    void StepAll(const InputTypes&... inputs){
        std::get<I>(*this).Step(std::get<I>(std::forward_as_tuple(inputs...)));
        if constexpr(I + 1 != sizeof...(InputTypes)){
            StepAll<I + 1>(inputs...);
        }
    }

    template<size_t I = 0, typename ...Parameters>
    void UpdateAllParameters(const Parameters&... parameters){
        std::get<I>(*this).SetParameters(std::get<I>(std::forward_as_tuple(parameters...)));
        if constexpr(I + 1 != sizeof...(Parameters)){
            UpdateAllParameters<I + 1>(parameters...);
        }
    }
};

}   // namespace gtfo