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

// Set operations such as intersection and union are used in boolean expressions. For convenience, they are defined
// in an enumeration
enum class Relation{
    Intersection,
    Union
};

// This class is a generic container where the objects are stored as if they are operands in a boolean expression.
// The intended usage is for situations where intersections and unions of such objects have certain meaning, so this
// container is just a way to store those objects and allow functions to act on them.
template <typename Operand>
class BooleanExpression{
public:
    // Inputs are copy constructed into the container, so they must be copy-constructible
    static_assert(std::is_copy_constructible_v<Operand>, "Operands to BooleanExpression must be copy-constructible.");

    using OperandPtr = std::shared_ptr<Operand>;
    using SubexpressionPtr = std::shared_ptr<BooleanExpression<Operand>>;

    // Explicitly indicate the copy constructor for clarity
    BooleanExpression(const BooleanExpression&) = default;

    // This is a general-purpose constructor that combines a bunch of operands and subexpressions with the same
    // set operation
    template <typename... InputTypes>
    BooleanExpression(const Relation& relation, const InputTypes&... inputs)
        : relation_(relation)
    {
        Insert(inputs...);
    }

    // This is the most way to insert objects into the boolean expression. Since the objects can be both operands, 
    // subexpressions, or any combination of those things, a parameter pack is used to match any combination. The 
    // specific cases are handled by each constexpr if block
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
                // Produce a false type which depends on the template argument:
                // https://devblogs.microsoft.com/oldnewthing/20200311-00/?p=103553
                static_assert(!sizeof(InputTypes), 
                    "Constructor arguments must inherit from either BooleanExpression or Operand, or be a std::vector of such types");
            }
        }(), ...);
    }

    // MapReduce is a way to apply a given function to every operand in the expression and combine the results to produce
    // a single final result. The caller supplies two lambdas: a map lambda and a reduce lambda. The map lambda converts an 
    // operand into a custom return type, and the reduce lambda takes a vector of return values and produces a single return
    // value. Many operations on the boolean expression can be written in a map and reduce way.
    template <typename ReturnType>
    ReturnType MapReduce(
        const std::function<ReturnType(const Operand&)>& map, 
        const std::function<ReturnType(const Relation&, const std::vector<ReturnType>&)>& reduce) const
    {
        // Preallocate since the number of operands and subexpressions are known
        std::vector<ReturnType> return_values;
        return_values.reserve(operands_.size() + subexpressions_.size());

        // Apply the map function on each operand and store their results into return_values
        std::transform(operands_.cbegin(), operands_.cend(), std::back_inserter(return_values), 
            [&map](const OperandPtr& operand_ptr)->ReturnType{
                return map(*operand_ptr);
            }
        );
        // Recurse through the subexpressions by calling MapReduce on them, and store their results into return_values
        std::transform(subexpressions_.cbegin(), subexpressions_.cend(), std::back_inserter(return_values), 
            [&](const SubexpressionPtr& subexpression_ptr)->ReturnType{
                return subexpression_ptr->MapReduce(map, reduce);
            }
        );
        // Finally call reduce one last time to combine all the return values into a single return value
        return reduce(relation_, return_values);
    }

protected:
    Relation relation_;
    std::vector<OperandPtr> operands_;
    std::vector<SubexpressionPtr> subexpressions_;
};

}   // namespace gtfo