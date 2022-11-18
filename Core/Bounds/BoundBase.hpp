//----------------------------------------------------------------------------------------------------
// File: BoundBase.hpp
// Desc: base class for bounds
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <algorithm>
#include <iterator>
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
        return ContainerContains(tree_, point);
    }

    virtual VectorN operator()(const VectorN& point, const VectorN& prev_point, const Scalar& tol = 0.01) const {
        assert(!tree_.empty());
        // It is assumed that prev_point is contained within the bound

        // Find the set of expressions that contain the previous point, but not the current one
        std::unordered_set<BoundPtr> violated_bounds;
        std::copy_if(tree_.begin(), tree_.end(), std::inserter(violated_bounds, violated_bounds.begin()), [&point](const BoundPtr& ptr)->bool{
            return !ptr->Contains(point);
        });

        // If that set is empty, then the point is already contained within the bound
        if(violated_bounds.empty()){
            return point;
        }

        // Otherwise, bisection search until:
        //   Union: at least one expression in the set contains the point
        //   Intersection: all expressions in the set contain the point
        const VectorN difference_vector = point - prev_point;
        const Scalar scale_opt = BisectionSearch([&](const Scalar& scale)->bool{
            const VectorN test_point = prev_point + scale * difference_vector;
            return ContainerContains(violated_bounds, test_point);
        }, tol);

        return prev_point + scale_opt * difference_vector;
    }

protected:

    Scalar BisectionSearch(std::function<bool(const Scalar&)> evaluator, const Scalar& tol) const {
        std::pair<Scalar, Scalar> search_interval = std::make_pair(0.0, 1.0);
        while((search_interval.second - search_interval.first) > tol){
            const Scalar scale = (search_interval.first + search_interval.second) / 2.0;
            if(evaluator(scale)){
                search_interval.first = scale;
            } else{
                search_interval.second = scale;
            }
        }
        return search_interval.first;
    }

private:

    template <typename Container>
    bool ContainerContains(const Container& container, const VectorN& point) const {
        if(relation_ == Relation::Union){
            return std::any_of(container.begin(), container.end(), [&point](const BoundPtr& ptr)->bool{
                return ptr->Contains(point);
            });
        } else{ // Relation::Intersection
            return std::all_of(container.begin(), container.end(), [&point](const BoundPtr& ptr)->bool{
                return ptr->Contains(point);
            });
        }
    }

    Relation relation_;
    std::vector<BoundPtr> tree_;
};

template <unsigned int Dimensions, typename Scalar = double>
class BoundBase : public BoundExpression<Dimensions, Scalar>{
public:
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    BoundBase() {}

    virtual bool Contains(const VectorN& point) const override {
        return true;
    }

    virtual VectorN operator()(const VectorN& point, const VectorN& prev_point, const Scalar& tol = 0.01) const override {
        // Assume that the previous point is within the bounds
        assert(Contains(prev_point));

        // Return if there is no bound violation
        if(Contains(point)){
            return point;
        }

        // Otherwise bisection search
        const VectorN difference_vector = point - prev_point;
        const Scalar scale_opt = BoundExpression<Dimensions, Scalar>::BisectionSearch([&](const Scalar& scale){
            const VectorN test_point = prev_point + scale * difference_vector;
            return Contains(test_point);
        }, tol);
        return prev_point + scale_opt * difference_vector;
    }
};

}   // namespace gtfo