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

namespace gtfo{

template <typename... Models>
class DynamicsSelector : public DynamicsBase<std::tuple_element<0, std::tuple<Models...>>::type::Dimension, typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType>{
public:
    using Scalar = typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType;
    using Base = DynamicsBase<std::tuple_element<0, std::tuple<Models...>>::type::Dimension, Scalar>;
    using VectorN = typename Base::VectorN;

    static_assert(std::conjunction_v<std::is_base_of<Base, Models>...>, "Models must inherit from DynamicsBase");
    static_assert(std::conjunction_v<std::is_same<Scalar, typename Models::ScalarType>...>, "Models must have the same Scalar type");
    static_assert(((Base::Dimension == Models::Dimension) && ...), "Models must have the same dimension");

    // Constructs a DynamicsSelector with the models passed in as arguments. Each model must inherit from DynamicsBase and have dimension equal to that of DynamicsSelector
    DynamicsSelector(const Models&... models)
        :   Base(),
            models_(models...),
            index_(0)
    {
        // Match the current state to the first model
        this->SyncSystemTo(std::get<0>(models_));
    }

    // Step ensures that only the dynamics of the selected model are propagated
    bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        // Update the active model
        Base* model = GetActiveModel();
        
        // Then ensure DynamicsSelector's state matches that of the active model
        bool result = model->Step(force_input, physical_position);
        this->SyncSystemTo(*model);
        return result;
    }

    void PauseDynamics(const bool& pause) override{
        GetActiveModel()->PauseDynamics(pause);
        Base::PauseDynamics(pause);
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

            new_model->SyncSystemTo(*old_model);
            this->SyncSystemTo(*new_model);

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

private:
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

    std::tuple<Models...> models_;
    size_t index_;
};

}   // namespace gtfo