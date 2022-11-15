//----------------------------------------------------------------------------------------------------
// File: BoundBase.hpp
// Desc: base class for bounds
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <algorithm>
#include <type_traits>
#include <memory>

// Third-party dependencies
#include <Eigen/Dense>

namespace gtfo {

template <unsigned int Dimensions, typename Scalar>
class BoundExpression{

    static_assert(Dimensions > 0, "Dimensions must be at least 1.");
    static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using BoundPtr = std::shared_ptr<const BoundExpression>;

private:
    enum class Relation{
        Union,
        Intersection
    };

public:
    BoundExpression()
        :   relation_(Relation::Union) {}

    BoundExpression(const Relation& relation)
        :   relation_(relation){}

    BoundExpression(const BoundExpression& bound_expression) = default;

    template <typename DerivedA, typename DerivedB>
    friend BoundExpression operator&(const DerivedA& lhs, const DerivedB& rhs){
        static_assert(std::is_base_of_v<BoundExpression, DerivedA> && std::is_base_of_v<BoundExpression, DerivedB>, "BoundExpression::operator& template arguments need to be a BoundExpression or a derived class of Bound");
        BoundExpression expression(Relation::Intersection);
        expression.tree_.push_back(std::make_shared<DerivedA>(lhs));
        expression.tree_.push_back(std::make_shared<DerivedB>(rhs));
        return expression;
    }

    template <typename DerivedA, typename DerivedB>
    friend BoundExpression operator|(const DerivedA& lhs, const DerivedB& rhs){
        static_assert(std::is_base_of_v<BoundExpression, DerivedA> && std::is_base_of_v<BoundExpression, DerivedB>, "BoundExpression::operator| template arguments need to be a BoundExpression or a derived class of Bound");
        BoundExpression expression(Relation::Union);
        expression.tree_.push_back(std::make_shared<DerivedA>(lhs));
        expression.tree_.push_back(std::make_shared<DerivedB>(rhs));
        return expression;
    }

    virtual bool Contains(const VectorN& point) const {
        assert(!tree_.empty());
        if(relation_ == Relation::Union){
            return std::any_of(tree_.begin(), tree_.end(), [&point](const BoundPtr& ptr){
                return ptr->Contains(point);
            });
        } else{ // Relation::Intersection
            return std::all_of(tree_.begin(), tree_.end(), [&point](const BoundPtr& ptr){
                return ptr->Contains(point);
            });
        }
    }

    virtual VectorN operator()(const VectorN& point) const {
        // TODO
        return point;
    }

private:
    Relation relation_;
    std::vector<BoundPtr> tree_;
};

template <unsigned int Dimensions, typename Scalar = double>
class BoundBase : public BoundExpression<Dimensions, Scalar>{
public:
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using BoundExpression = BoundExpression<Dimensions, Scalar>;

    BoundBase() {}

    virtual bool Contains(const VectorN& point) const override {
        return true;
    }

    virtual VectorN operator()(const VectorN& point) const override {
        return point;
    }
};

}   // namespace gtfo