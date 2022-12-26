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
    using SubexpressionPtr = std::shared_ptr<BooleanExpression<Operand>>;

    BooleanExpression(const BooleanExpression&) = default;

    template <typename... InputTypes>
    BooleanExpression(const Relation& relation, const InputTypes&... inputs)
        : relation_(relation)
    {
        Insert(inputs...);
    }

    template <typename... InputTypes>
    void Insert(const InputTypes&... inputs){
        ([&]{
            // Input is / inherits from a BooleanExpression 
            if constexpr(std::is_base_of_v<BooleanExpression, InputTypes>)
            {
                if(inputs.relation_ == relation_){
                    std::copy(inputs.subexpressions_.cbegin(), inputs.subexpressions_.cend(), std::back_inserter(subexpressions_));
                    std::copy(inputs.operands_.cbegin(), inputs.operands_.cend(), std::back_inserter(operands_));
                } else{
                    subexpressions_.push_back(std::static_pointer_cast<BooleanExpression>(std::make_shared<InputTypes>(inputs)));
                }
            } 
            // Input is / inherits from an Operand
            else if constexpr(std::is_base_of_v<Operand, InputTypes> || std::is_same_v<Operand, InputTypes>)
            {
                operands_.push_back(std::static_pointer_cast<Operand>(std::make_shared<InputTypes>(inputs)));
            } 
            // Input is a std::vector of a type that is / inherits from a BooleanExpression
            else if constexpr(std::is_same_v<InputTypes, std::vector<typename InputTypes::value_type>> && 
                std::is_base_of_v<BooleanExpression, typename InputTypes::value_type>)
            {
                for(const typename InputTypes::value_type& input : inputs){
                    if(input.relation_ == relation_){
                        std::copy(input.subexpressions_.cbegin(), input.subexpressions_.cend(), std::back_inserter(subexpressions_));
                        std::copy(input.operands_.cbegin(), input.operands_.cend(), std::back_inserter(operands_));
                    } else{
                        subexpressions_.push_back(std::static_pointer_cast<BooleanExpression>(std::make_shared<InputTypes::value_type>(input)));
                    }
                }
            }
            // Input is a std::vector of a type that is / inherits from an Operand
            else if constexpr(std::is_same_v<InputTypes, std::vector<typename InputTypes::value_type>> && 
                (std::is_base_of_v<Operand, typename InputTypes::value_type> || std::is_same_v<Operand, typename InputTypes::value_type>))
            {
                std::transform(inputs.cbegin(), inputs.cend(), std::back_inserter(operands_), 
                    [](const typename InputTypes::value_type& input)->OperandPtr{
                        return std::static_pointer_cast<Operand>(std::make_shared<InputTypes::value_type>(input));
                    }
                );
            }
            // Otherwise produce an error
            else
            {
                static_assert(!sizeof(InputTypes), 
                    "Constructor arguments must inherit from either BooleanExpression or Operand, or be a std::vector of such types");
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
            [&map](const OperandPtr& operand_ptr)->ReturnType{
                return map(*operand_ptr);
            }
        );
        std::transform(subexpressions_.cbegin(), subexpressions_.cend(), std::back_inserter(return_values), 
            [&](const SubexpressionPtr& subexpression_ptr)->ReturnType{
                return subexpression_ptr->MapReduce(map, reduce);
            }
        );

        return reduce(relation_, return_values);
    }

protected:
    Relation relation_;
    std::vector<OperandPtr> operands_;
    std::vector<SubexpressionPtr> subexpressions_;
};

}   // namespace gtfo