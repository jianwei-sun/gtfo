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


    // TODO: create mega constructor
    // static_cast(std::is_same<T, std::vector<typename T::value_type>>::value);
    // template <typename... InputTypes, typename = 
    //     typename std::enable_if_t<(true && ... &&
    //         (std::is_base_of_v<BooleanExpression, InputTypes> ||
    //         std::is_base_of_v<Operand, InputTypes> ||
    //         std::is_same_v<Operand, InputTypes>)), void
    //     >
    // >
    // BooleanExpression(const Relation& relation, InputTypes&... inputs)
    //     : relation_(relation)
    // {    
    //     ([&]{
    //         if constexpr(std::is_base_of_v<BooleanExpression, InputTypes>){
    //             if(inputs.relation_ == relation_){
    //                 std::copy(inputs.subexpressions_.cbegin(), inputs.subexpressions_.cend(), std::back_inserter(subexpressions_));
    //                 std::copy(inputs.operands_.cbegin(), inputs.operands_.cend(), std::back_inserter(operands_));
    //             } else{
    //                 subexpressions_.push_back(inputs);
    //             }
    //         } else if constexpr(std::is_base_of_v<Operand, InputTypes> || std::is_same_v<Operand, InputTypes>){
    //             operands_.push_back(std::static_pointer_cast<Operand>(std::make_shared<InputTypes>(inputs)));
    //         } else{
    //             static_assert(!sizeof(InputTypes), "Constructor arguments must inherit from either BooleanExpression or Operand.");
    //         }
    //     }(), ...);
    // }

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
                    subexpressions_.push_back(inputs);
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
                for(const BooleanExpression& input : inputs){
                    if(input.relation_ == relation_){
                        std::copy(input.subexpressions_.cbegin(), input.subexpressions_.cend(), std::back_inserter(subexpressions_));
                        std::copy(input.operands_.cbegin(), input.operands_.cend(), std::back_inserter(operands_));
                    } else{
                        subexpressions_.push_back(input);
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
                    "Constructor arguments must inherit from either BooleanExpression or Operand, or be a std::vector of such type");
            }
        }(), ...);
    }

    // template <typename InputType>
    // BooleanExpression(
    //     const Relation& relation, 
    //     const std::enable_if_t<
    //         std::is_base_of_v<BooleanExpression, InputType> ||
    //         std::is_base_of_v<Operand, InputType> ||
    //         std::is_same_v<Operand, InputType>, 
    //     std::vector<InputType>>& inputs)
    //     : relation_(relation)
    // {
    //     if constexpr(std::is_base_of_v<BooleanExpression, InputType>){
    //         for(const BooleanExpression& input : inputs){
    //             if(input.relation_ == relation_){
    //                 std::copy(input.subexpressions_.cbegin(), input.subexpressions_.cend(), std::back_inserter(subexpressions_));
    //                 std::copy(input.operands_.cbegin(), input.operands_.cend(), std::back_inserter(operands_));
    //             } else{
    //                 subexpressions_.push_back(input);
    //             }
    //         }
    //     } else if constexpr(std::is_base_of_v<Operand, InputType> || std::is_same_v<Operand, InputType>){
    //         std::transform(inputs.cbegin(), inputs.cend(), std::back_inserter(operands_), 
    //             [](const InputType& input)->OperandPtr{
    //                 return std::static_pointer_cast<Operand>(std::make_shared<InputType>(input));
    //             }
    //         );
    //     }
    // }

    // template <typename InputType>
    // void PushBack(
    //     const std::enable_if_t<
    //         std::is_base_of_v<BooleanExpression, InputType> ||
    //         std::is_base_of_v<Operand, InputType> ||
    //         std::is_same_v<Operand, InputType>, 
    //     InputType>& input)
    // {
    //     if constexpr(std::is_base_of_v<BooleanExpression, InputType>){
    //         if(input.relation_ == relation_){
    //             std::copy(input.subexpressions_.cbegin(), input.subexpressions_.cend(), std::back_inserter(subexpressions_));
    //             std::copy(input.operands_.cbegin(), input.operands_.cend(), std::back_inserter(operands_));
    //         } else{
    //             subexpressions_.push_back(input);
    //         }
    //     } else if constexpr(std::is_base_of_v<Operand, InputType> || std::is_same_v<Operand, InputType>){
    //         operands_.push_back(std::static_pointer_cast<Operand>(std::make_shared<InputType>(input)));
    //     }
    // }

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