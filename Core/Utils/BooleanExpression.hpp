//----------------------------------------------------------------------------------------------------
// File: BooleanExpression.hpp
// Desc: base class for the boolean expression container
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <algorithm>
#include <iterator>
#include <type_traits>
#include <vector>
#include <functional>


namespace gtfo {

enum class Relation{
    Intersection,
    Union
};

template <typename Item>
class BooleanExpression{

public:
    // BooleanExpression(const Relation& relation) : relation_(relation) {}

    BooleanExpression(const BooleanExpression&) = default;

    template <typename... Types>
    BooleanExpression(const Relation& relation, const Types&... types)
        : relation_(relation)
    {    
        ([&]{
            if constexpr(std::is_base_of_v<BooleanExpression, Types>){
                if(types.relation_ == relation_){
                    std::copy(types.subexpressions_.begin(), types.subexpressions_.end(), std::back_inserter(subexpressions_));
                    std::copy(types.items_.begin(), types.items_.end(), std::back_inserter(items_));
                } else{
                    subexpressions_.push_back(types);
                }
            } else if constexpr(std::is_same_v<Item, Types>){
                items_.push_back((types));
            } else{
                static_assert(!sizeof(Types), "Argument types must inherit from either BooleanExpression or Item.");
            }
        }(), ...);
    }

    // template <typename A, typename B>
    // friend BooleanExpression operator&(const A& lhs, const B& rhs){
    //     return BooleanExpression(Relation::Intersection, lhs, rhs);
    // }

    // template <typename A, typename B>
    // friend BooleanExpression operator|(const A& lhs, const B& rhs){
    //     return BooleanExpression(Relation::Union, lhs, rhs);
    // }

    // template <typename LeftType, typename RightType>
    // friend typename std::enable_if_t<!std::is_base_of_v<Item, LeftType> || !std::is_base_of_v<Item, RightType>, BooleanExpression> 
    // operator&(const LeftType& lhs, const RightType& rhs){
    //     return BooleanExpression(Relation::Intersection, lhs, rhs);
    // }

    template <typename ReturnType>
    ReturnType MapReduce(const std::function<ReturnType(const Item&)>& map, const std::function<ReturnType(const Relation&, const std::vector<ReturnType>&)>& reduce) const{
        std::vector<ReturnType> return_values;
        return_values.reserve(items_.size() + subexpressions_.size());

        std::transform(items_.cbegin(), items_.cend(), std::back_inserter(return_values), map);
        std::transform(subexpressions_.cbegin(), subexpressions_.cend(), std::back_inserter(return_values), [&](const BooleanExpression<Item>& subexpression)->ReturnType{
            return subexpression.MapReduce(map, reduce);
        });
        return reduce(relation_, return_values);

        // std::transform_reduce(
        //     items_.begin(), items_.end(),
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
    std::vector<Item> items_;
    std::vector<BooleanExpression<Item>> subexpressions_;
    
};

}   // namespace gtfo