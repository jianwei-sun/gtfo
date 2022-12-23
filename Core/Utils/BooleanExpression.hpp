//----------------------------------------------------------------------------------------------------
// File: BooleanExpression.hpp
// Desc: base class for the boolean expression container
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <iterator>
#include <type_traits>
#include <vector>

namespace gtfo {

enum class Relation{
    Intersection,
    Union
};

template <typename Item>
class BooleanExpression{

public:
    // BooleanExpression(const Relation& relation) : relation_(relation) {}

    template <typename... Types>
    BooleanExpression(const Relation& relation, const Types&... types)
        : relation_(relation)
    {    
        ([&]{
            if constexpr(std::is_same_v<Types, BooleanExpression>){
                if(types.relation_ == relation_){
                    std::copy(types.subexpressions_.begin(), types.subexpressions_.end(), std::back_inserter(subexpressions_));
                    std::copy(types.items_.begin(), types.items_.end(), std::back_inserter(items_));
                } else{
                    subexpressions_.push_back(types);
                }
            } else if constexpr(std::is_same_v<Types, Item>){
                items_.push_back(types);
            } else{
                static_assert(!sizeof(Types), "Argument types must be either BooleanExpression or Item.");
            }
        }(), ...);
    }

    template <typename A, typename B>
    friend BooleanExpression operator&(const A& lhs, const B& rhs){
        return BooleanExpression(Relation::Intersection, lhs, rhs);
    }

    template <typename A, typename B>
    friend BooleanExpression operator|(const A& lhs, const B& rhs){
        return BooleanExpression(Relation::Union, lhs, rhs);
    }

private:

    Relation relation_;
    std::vector<BooleanExpression<Item>> subexpressions_;
    std::vector<Item> items_;

};

}   // namespace gtfo