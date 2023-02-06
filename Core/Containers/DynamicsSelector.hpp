//----------------------------------------------------------------------------------------------------
// File: DynamicsSelector.hpp
// Desc: reference class, which serves as a multiplexer for selecting between a set of models
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <tuple>
#include <type_traits>
#include <memory>
#include <vector>

// Project-specific
#include "../Models/DynamicsBase.hpp"

namespace gtfo{

template <unsigned int Dimensions, typename Scalar = double>
class DynamicsSelector : public DynamicsBase<Dimensions, Scalar>, private std::vector<std::shared_ptr<DynamicsBase<Dimensions, Scalar>>>{
public:
    using Base = DynamicsBase<Dimensions, Scalar>;
    using ModelPtr = std::shared_ptr<Base>;
    using Container = std::vector<ModelPtr>;

    using VectorN = typename Base::VectorN;

    // Constructs a DynamicsSelector with the models passed in as arguments. Each model must inherit from DynamicsBase and have dimension equal to that of DynamicsSelector
    template <typename... Models>
    DynamicsSelector(const Models&... models)
        :   Base(/*std::get<0>(std::forward_as_tuple(models...)).GetPosition()*/),
            Container({std::make_shared<Base>(models)...}),
            index_(0)
    {
        static_assert(std::conjunction_v<std::is_base_of<Base, Models>...>, "Input types must inherit from DynamicsBase");
        static_assert(((Dimensions == Models::Dimension) && ...), "Each model's dimension must equal DynamicsSelector dimension");

        // Ensure at least one model is present
        assert(Container::size() > 0);

        // // Match the current state to the first model
        // Base::position_ = Container::operator[](0).GetPosition();
        // Base::velocity_ = Container::operator[](0).GetVelocity();
        // Base::acceleration_ = Container::operator[](0).GetAcceleration();
    }

    // Step ensures that only the dynamics of the selected model are propagated
    bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        ModelPtr& model_ptr = Container::operator[](index_);
        const bool result = model_ptr->Step(force_input, physical_position);

        // Base::position_ = model_ptr->GetPosition();
        // Base::velocity_ = model_ptr->GetVelocity();
        // Base::acceleration_ = model_ptr->GetAcceleration();

        return result;
    }

    void PauseDynamics(const bool& pause) override{
        Container::operator[](index_)->PauseDynamics(pause);
        Base::PauseDynamics(pause);
    }

    [[nodiscard]] inline const VectorN &GetPosition() const override
    {
        return Container::operator[](index_)->GetPosition();
    }

    [[nodiscard]] inline const VectorN &GetVelocity() const override
    {
        return Container::operator[](index_)->GetVelocity();
    }

    [[nodiscard]] inline const VectorN &GetAcceleration() const override
    {
        return Container::operator[](index_)->GetAcceleration();
    }

    // Selects a model using an index, and returns true if the index is within bounds. The newly selected
    // model's state is also updated to equal that of the previously selected model. However, the newly 
    // selected model's bounds are also checked, which may result in discontinuities in the state
    bool Select(const size_t& index){
        if(index < Container::size()){
            // Update the newly selected model's state to match the previous model's state
            ModelPtr& old_model_ptr = Container::operator[](index_);
            ModelPtr& new_model_ptr = Container::operator[](index);

            new_model_ptr->position_ = old_model_ptr->position_;
            new_model_ptr->velocity_ = old_model_ptr->velocity_;
            new_model_ptr->EnforceHardBound();
            new_model_ptr->EnforceVelocityLimit();
            new_model_ptr->acceleration_ = old_model_ptr->acceleration_;

            new_model_ptr->dynamics_paused_ = old_model_ptr->dynamics_paused_;

            // // Ensure the DynamicsSelector's state matches those of the newly selected model
            // Base::position_ = new_model_ptr->GetPosition();
            // Base::velocity_ = new_model_ptr->GetVelocity();
            // Base::acceleration_ = new_model_ptr->GetAcceleration();

            index_ = index;
            return true;
        } else{
            return false;
        }
    }
    
    // Gets the index of the current selection
    size_t CurrentSelection(void) const{
        return index_;
    }

private:
    size_t index_;

};

}   // namespace gtfo