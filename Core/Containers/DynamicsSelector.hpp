//----------------------------------------------------------------------------------------------------
// File: DynamicsSelector.hpp
// Desc: reference class, which serves as a multiplexer for selecting between a set of models
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <tuple>
#include <type_traits>
#include <memory>

// Project-specific
#include "../Models/DynamicsBase.hpp"
#include "../Bounds/BoundBase.hpp"

namespace gtfo{

template <typename... Models>
class DynamicsSelector : public DynamicsBase<
    std::tuple_element<0, std::tuple<Models...>>::type::Dimension, 
    typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType, 
    std::tuple_element<0, std::tuple<Models...>>::type::PositionDimension>{
public:
    using Scalar = typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType;
    using Base = DynamicsBase<std::tuple_element<0, std::tuple<Models...>>::type::Dimension, Scalar, std::tuple_element<0, std::tuple<Models...>>::type::PositionDimension>;
    using Bound = BoundBase<std::tuple_element<0, std::tuple<Models...>>::type::PositionDimension, Scalar>;
    using VectorN = typename Base::VectorN;
    using VectorP = typename Base::VectorP;

    static_assert(std::conjunction_v<std::is_base_of<Base, Models>...>, "Models must inherit from DynamicsBase");
    static_assert(std::conjunction_v<std::is_same<Scalar, typename Models::ScalarType>...>, "Models must have the same Scalar type");
    static_assert(((Base::Dimension == Models::Dimension) && ...), "Models must have the same dimension");
    static_assert(((Base::PositionDimension == Models::PositionDimension) && ...), "Models must have the same position dimension");

    // Constructs a DynamicsSelector with the models passed in as arguments. Each model must inherit from DynamicsBase and have dimension equal to that of DynamicsSelector
    DynamicsSelector(const Models&... models)
        :   Base(),
            models_(models...),
            index_(0)
    {}

    // PropagateDynamics is empty since each model already contains the relevant dynamics
    void PropagateDynamics(const VectorN& force_input) override{
        return;
    }

    // Step ensures that only the dynamics of the selected model are propagated
    void Step(const VectorN& force_input, const VectorP& physical_position = VectorP::Constant(NAN)) override{     
        GetActiveModel()->Step(force_input, physical_position);
    }

    [[nodiscard]] VectorP GetPosition(void) const override{
        return GetActiveModel()->GetPosition();
    }

    [[nodiscard]] VectorP GetOldPosition(void) const override{
        return GetActiveModel()->GetOldPosition();
    }

    [[nodiscard]] VectorN GetVelocity(void) const override{
        return GetActiveModel()->GetVelocity();
    }

    [[nodiscard]] VectorN GetAcceleration(void) const override{
        return GetActiveModel()->GetAcceleration();
    }

    [[nodiscard]] bool DynamicsArePaused(void) const override{
        return GetActiveModel()->DynamicsArePaused();
    }

    void SetFullState(const VectorP& position, const VectorP& old_position, const VectorN& velocity, const VectorN& acceleration, const bool& dynamics_paused) override{
        GetActiveModel()->SetFullState(
            position, 
            old_position,
            velocity,
            acceleration,
            dynamics_paused
        );
    }

    void SetState(const VectorP& position, const VectorN& velocity, const VectorN& acceleration) override{
        GetActiveModel()->SetState(position, velocity, acceleration);
    }

    void PauseDynamics(const bool& pause) override{
        GetActiveModel()->PauseDynamics(pause);
    }

    // Bounds are prevented from being set at the container level. They should be set at the submodel level
    void SetHardBound(const Bound& bound) override{
        assert(false);
    }

    void SetSoftBound(const Bound& bound, const Scalar &spring_constant, const Scalar &damping_constant) override{
        assert(false);
    }

    // Selects a model using an index, and returns true if the index is within bounds. The newly selected
    // model's state is also updated to equal that of the previously selected model. However, the newly 
    // selected model's bounds are also checked, which may result in discontinuities in the state
    template <typename T>
    bool Select(const T& selection){
        const size_t index = static_cast<size_t>(selection);
        if(index != index_ && index < std::tuple_size_v<std::tuple<Models...>>){
            Base* old_model = GetActiveModel();
            index_ = index;
            Base* new_model = GetActiveModel();
            new_model->SyncModelTo(*old_model);
            return true;
        } else{
            return false;
        }
    }
    
    // Gets the index of the current selection
    size_t CurrentSelection(void) const{
        return index_;
    }

    // Getter methods for models
    template <size_t I>
    typename std::tuple_element<I, std::tuple<Models...>>::type& GetModel(void) {
        return std::get<I>(models_);
    }

    template <size_t I>
    const typename std::tuple_element<I, std::tuple<Models...>>::type& GetModel(void) const{
        return std::get<I>(models_);
    }

    // Helper function for retrieving the currently active model at runtime, by indexing the tuple
    // using template recursion. Since all models inherit from the same base type under this class,
    // their pointers can be casted to the base type
    template <size_t I = 0>
    Base* GetActiveModel(){
        if(I == index_){
            return &std::get<I>(models_);
        } else if constexpr(I + 1 < std::tuple_size_v<std::tuple<Models...>>){
            return GetActiveModel<I + 1>();
        } else{
            return nullptr;
        }
    }

    template <size_t I = 0>
    const Base* GetActiveModel() const{
        if(I == index_){
            return &std::get<I>(models_);
        } else if constexpr(I + 1 < std::tuple_size_v<std::tuple<Models...>>){
            return GetActiveModel<I + 1>();
        } else{
            return nullptr;
        }
    }

private:
    std::tuple<Models...> models_;
    size_t index_;
};

}   // namespace gtfo