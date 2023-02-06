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
#include <vector>

// Project-specific
#include "../Models/DynamicsBase.hpp"

namespace gtfo{

template <unsigned int Dimensions, typename Scalar = double>
class DynamicsVector : public DynamicsBase<Dimensions, Scalar>, private std::vector<std::shared_ptr<DynamicsBase<Dimensions, Scalar>>>{
public:
    using Base = DynamicsBase<Dimensions, Scalar>;
    using ModelPtr = std::shared_ptr<Base>;
    using Container = std::vector<ModelPtr>;

    using VectorN = typename Base::VectorN;

    // Constructs a DynamicsVector with the models passed in as arguments. Each model must inherit from DynamicsBase. 
    // The sum of dimensions of the argument models must equal the dimension of DynamicsVector
    template <typename... Models>
    DynamicsVector(const Models&... models)
        :   Base((VectorN() << ... << models.GetPosition()).finished()),
            Container({std::make_shared<Base>(models)...})
    {
        static_assert(std::conjunction_v<std::is_base_of<Base, Models>...>, "Input types must inherit from DynamicsBase");
        static_assert(Dimensions == (Models::Dimension + ...), "DynamicsVector dimension must equal sum of model dimensions");

        // Ensure at least one model is present
        assert(Container::size() > 0);

        // Match the current state to the first model
        Base::position_ = (VectorN() << ... << models.GetPosition()).finished();
        Base::velocity_ = (VectorN() << ... << models.GetVelocity()).finished();
        Base::acceleration_ = (VectorN() << ... << models.GetAcceleration()).finished();
    }

    // Step allows stepping all the models in DynamicsVector as if the container is a single model. Step only
    // returns true if all models' Step functions return true
    bool Step(const VectorN& force_input, const VectorN& physical_position = VectorN::Constant(NAN)) override{
        bool result = true;
        unsigned int index = 0;
        for(ModelPtr& model : *this){
            result &= model->Step(force_input.block(index, 0, model->Dimension, 1), physical_position.block(index, 0, model->Dimension, 1));

            Base::position_.block(index, 0, model->Dimension, 1) = model->GetPosition();
            Base::velocity_.block(index, 0, model->Dimension, 1) = model->GetVelocity();
            Base::acceleration_.block(index, 0, model->Dimension, 1) = model->GetAcceleration();

            index += model->Dimension;
        }
        return result;
    }

    void PauseDynamics(const bool& pause) override{
        for(ModelPtr& model : *this){
            model->PauseDynamics(pause);
        }
        Base::PauseDynamics(pause);
    }

    // Note that just like std::vector::operator[], indices are not checked against bounds. So, out of range
    // access will result in undefined behavior. Furthermore, the function returns a reference to the pointer,
    // so a static cast is necessary for accessing members in subclasses of DynamicsBase
    ModelPtr& operator[](const size_t& index) override{
        return Container::operator[](index);
    }
};

}   // namespace gtfo