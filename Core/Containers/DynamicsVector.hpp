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
#include <iostream>
// Project-specific
#include "../Models/DynamicsBase.hpp"

namespace gtfo{

template <typename... Models>
class DynamicsVector : public DynamicsBase<(Models::Dimension + ...), typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType>{
public:
    using Scalar = typename std::tuple_element<0, std::tuple<Models...>>::type::ScalarType;
    using Base = DynamicsBase<(Models::Dimension + ...), Scalar>;
    using VectorN = typename Base::VectorN;

    // Constructs a DynamicsVector with the models passed in as arguments. Each model must inherit from DynamicsBase. 
    DynamicsVector(const Models&... models)
        :   Base(),
            models_(models...)
    {
        static_assert(std::conjunction_v<std::is_base_of<DynamicsBase<Models::Dimension, Scalar>, Models>...>, "Input types must inherit from DynamicsBase");

        // Update the state variables of DynamicsVector. Use an index to keep track of where each model's
        // dimensions begin
        size_t index = 0;
        ([&]{
            Base::position_.block<Models::Dimension, 1>(index, 0) = models.GetPosition();
            Base::velocity_.block<Models::Dimension, 1>(index, 0) = models.GetVelocity();
            Base::acceleration_.block<Models::Dimension, 1>(index, 0) = models.GetAcceleration();
            index += Models::Dimension;
        }(), ...);
        std::cout << "Constructed a DynamicsVector of dimension " << (Models::Dimension + ...) << "\n";


    }

    // DynamicsVector(const DynamicsVector& other)
    //     :   Base(other)
    // {
    //     std::cout << "called copy constructor\n";
    //     std::apply([&](auto&&... model){
    //         // ([&]{
    //         //     std::make_shared(*model)
    //         // }(), ...);

    //         models_ = std::forward_as_tuple(std::make_shared<DynamicsBase<Dimensions, Scalar>>(*model)...);
    //     }, other.models_);
    // }

    // DynamicsVector(DynamicsVector&& other){
    //     std::cout << "called move constructor\n";
    //     models_ = other.models_;
    // }

    // DynamicsVector& operator=(const DynamicsVector& other){
    //     std::cout << "called copy assignment\n";
    //     if(other == *this){
    //         return *this;
    //     }
    //     std::apply([&](auto&&... model){
    //         // ([&]{
    //         //     std::make_shared(*model)
    //         // }(), ...);

    //         models_ = std::forward_as_tuple(std::make_shared<DynamicsBase<Dimensions, Scalar>>(*model)...);
    //     }, other.models_);
    // }

    // DynamicsVector& operator=(DynamicsVector&& other){
    //     std::cout << "called move assignment\n";
    //     models_ = other.models_;
    // }

    // Step allows stepping all the models in DynamicsVector as if the container is a single model. Step only
    // returns true if all models' Step functions return true
    bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        bool result = true;

        std::apply([&](auto&&... model){
            // Index is used to keep track of where each model's dimensions begin in the concatenated VectorN
            size_t index = 0;

            // First, step all the models with the appropriate coordinates of the inputs
            ([&]{
                result &= model.Step(force_input.block<Models::Dimension, 1>(index, 0), physical_position.block<Models::Dimension, 1>(index, 0));
                index += Models::Dimension;
            }(), ...);

            // Then, update the state variables of DynamicsVector accordingly
            index = 0;
            ([&]{
                Base::position_.block<Models::Dimension, 1>(index, 0) = model.GetPosition();
                Base::velocity_.block<Models::Dimension, 1>(index, 0) = model.GetVelocity();
                Base::acceleration_.block<Models::Dimension, 1>(index, 0) = model.GetAcceleration();
                index += Models::Dimension;
            }(), ...);
        }, models_);
        
        return result;
    }

    void PauseDynamics(const bool& pause) override{
        std::apply([&](auto&&... model){
            (model.PauseDynamics(pause), ...);
        }, models_);
        Base::PauseDynamics(pause);
    }

    // Note that just like std::vector::operator[], indices are not checked against bounds. So, out of range
    // access will result in undefined behavior. Furthermore, the function returns a reference to the pointer,
    // so a static cast is necessary for accessing members in subclasses of DynamicsBase
    // ModelPtr& operator[](const size_t& index){
    //     return Container::operator[](index);
    // }

private:
    // std::tuple<std::shared_ptr<DynamicsBase<Dimensions, Scalar>>...> models_;
    std::tuple<Models...> models_;
};

}   // namespace gtfo