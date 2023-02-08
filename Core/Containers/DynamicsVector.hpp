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

template <typename Scalar, unsigned int... Dimensions>
class DynamicsVector : public DynamicsBase<(Dimensions + ...), Scalar>{
public:
    using Base = DynamicsBase<(Dimensions + ...), Scalar>;
    using VectorN = typename Base::VectorN;

    // Constructs a DynamicsVector with the models passed in as arguments. Each model must inherit from DynamicsBase. 
    // The sum of dimensions of the argument models must equal the dimension of DynamicsVector
    template <typename... Models>
    DynamicsVector(const Models&... models)
        :   Base(),
            models_(std::forward_as_tuple(std::make_shared<Models>(models)...))
    {
        static_assert(std::conjunction_v<std::is_base_of<DynamicsBase<Dimensions, Scalar>, Models>...>, "Input types must inherit from DynamicsBase");

        // Update the state variables of DynamicsVector. Use an index to keep track of where each model's
        // dimensions begin
        size_t index = 0;
        ([&]{
            Base::position_.block<Dimensions, 1>(index, 0) = models.GetPosition();
            Base::velocity_.block<Dimensions, 1>(index, 0) = models.GetVelocity();
            Base::acceleration_.block<Dimensions, 1>(index, 0) = models.GetAcceleration();
            index += Dimensions;
        }(), ...);
        std::cout << "Constructed a DynamicsVector of dimension " << (Dimensions + ...) << "\n";
    }

    DynamicsVector(const DynamicsVector& other)
        :   Base(other)
    {
        // models_ = std::make_shared
    }

    DynamicsVector(DynamicsVector&& other){

    }

    DynamicsVector& operator=(const DynamicsVector& other){

    }

    DynamicsVector& operator=(DynamicsVector&& other){

    }

    // Step allows stepping all the models in DynamicsVector as if the container is a single model. Step only
    // returns true if all models' Step functions return true
    bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        bool result = true;

        std::apply([&](auto&&... model){
            // Index is used to keep track of where each model's dimensions begin in the concatenated VectorN
            size_t index = 0;

            // First, step all the models with the appropriate coordinates of the inputs
            ([&]{
                result &= model->Step(force_input.block<Dimensions, 1>(index, 0), physical_position.block<Dimensions, 1>(index, 0));
                index += Dimensions;
            }(), ...);

            // Then, update the state variables of DynamicsVector accordingly
            index = 0;
            ([&]{
                Base::position_.block<Dimensions, 1>(index, 0) = model->GetPosition();
                Base::velocity_.block<Dimensions, 1>(index, 0) = model->GetVelocity();
                Base::acceleration_.block<Dimensions, 1>(index, 0) = model->GetAcceleration();
                index += Dimensions;
            }(), ...);
        }, models_);
        
        return result;
    }

    void PauseDynamics(const bool& pause) override{
        std::apply([&](auto&&... model){
            (model->PauseDynamics(pause), ...);
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
    std::tuple<std::shared_ptr<DynamicsBase<Dimensions, Scalar>>...> models_;

};

}   // namespace gtfo