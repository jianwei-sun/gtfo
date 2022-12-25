//----------------------------------------------------------------------------------------------------
// File: BooleanExpression.hpp
// Desc: base class for the boolean expression container
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <algorithm>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>
#include <functional>


namespace gtfo {

enum class Relation{
    Intersection,
    Union
};

template <typename Operand>
class BooleanExpression{
public:
    static_assert(std::is_copy_constructible_v<Operand>, "Operands to BooleanExpression must be copy-constructible.");

    using OperandPtr = std::shared_ptr<Operand>;
    using Subexpression = BooleanExpression<Operand>;

    // BooleanExpression(const Relation& relation) : relation_(relation) {}

    BooleanExpression(const BooleanExpression&) = default;

    template <typename... InputTypes>
    BooleanExpression(const Relation& relation, const InputTypes&... inputs)
        : relation_(relation)
    {    
        ([&]{
            if constexpr(std::is_base_of_v<BooleanExpression, InputTypes>){
                if(inputs.relation_ == relation_){
                    std::copy(inputs.subexpressions_.begin(), inputs.subexpressions_.end(), std::back_inserter(subexpressions_));
                    std::copy(inputs.operands_.begin(), inputs.operands_.end(), std::back_inserter(operands_));
                } else{
                    subexpressions_.push_back(inputs);
                }
            } else if constexpr(std::is_base_of_v<Operand, InputTypes> || std::is_same_v<Operand, InputTypes>){
                operands_.push_back(std::static_pointer_cast<Operand>(std::make_shared<InputTypes>(inputs)));
            } else{
                static_assert(!sizeof(InputTypes), "Constructor arguments must inherit from either BooleanExpression or Operand.");
            }
        }(), ...);
    }

    template <typename ReturnType>
    ReturnType MapReduce(
        const std::function<ReturnType(const Operand&)>& map, 
        const std::function<ReturnType(const Relation&, const std::vector<ReturnType>&)>& reduce) const
    {
        std::vector<ReturnType> return_values;
        return_values.reserve(operands_.size() + subexpressions_.size());

        std::transform(operands_.cbegin(), operands_.cend(), std::back_inserter(return_values), 
            [&map](const OperandPtr& ptr)->ReturnType{
                return map(*ptr);
            }
        );
        std::transform(subexpressions_.cbegin(), subexpressions_.cend(), std::back_inserter(return_values), 
            [&](const Subexpression& subexpression)->ReturnType{
                return subexpression.MapReduce(map, reduce);
            }
        );

        return reduce(relation_, return_values);

        // std::transform_reduce(
        //     operands_.begin(), operands_.end(),
        //     ReturnType{},
        //     reduce,
        //     map
        // );
        // std::transform_reduce(
        //     subexpressions_.begin(), subexpressions_.end(),
        //     ReturnType{},
        //     reduce,

        // )
    }

protected:

    Relation relation_;
    std::vector<OperandPtr> operands_;
    std::vector<Subexpression> subexpressions_;
    
};

}   // namespace gtfo