//----------------------------------------------------------------------------------------------------
// File: BoundExpression.hpp
// Desc: base class for boolean expressions of bounds
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <algorithm>
#include <iterator>
#include <type_traits>
#include <memory>
#include <unordered_set>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Utils/Constants.hpp"

template <unsigned int Dimensions, typename Scalar>
class BoundExpression{

    static_assert(Dimensions > 0, "Dimensions must be at least 1.");
    static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using BoundPtr = std::shared_ptr<const BoundExpression>;

private:
    enum class Relation{
        Intersection,
        Union
    };

public:
    BoundExpression() : relation_(Relation::Intersection) {}
    BoundExpression(const Relation& relation) : relation_(relation) {}

    template <typename DerivedA, typename DerivedB>
    friend BoundExpression operator&(const DerivedA& lhs, const DerivedB& rhs){
        static_assert(std::is_base_of_v<BoundExpression, DerivedA> && std::is_base_of_v<BoundExpression, DerivedB>, "BoundExpression::operator& template arguments need to be a BoundExpression or a derived class of Bound");
        BoundExpression expression(Relation::Intersection);

        if(std::is_same_v<DerivedA, BoundExpression> && lhs.relation_ == Relation::Intersection){
            std::copy(lhs.tree_.begin(), lhs.tree_.end(), std::back_inserter(expression.tree_));
        } else{
            expression.tree_.push_back(std::make_shared<const DerivedA>(lhs));
        } 

        if(std::is_same_v<DerivedB, BoundExpression> && rhs.relation_ == Relation::Intersection){
            std::copy(rhs.tree_.begin(), rhs.tree_.end(), std::back_inserter(expression.tree_));
        } else{
            expression.tree_.push_back(std::make_shared<const DerivedB>(rhs));
        }    

        return expression;
    }

    template <typename DerivedA, typename DerivedB>
    friend BoundExpression operator|(const DerivedA& lhs, const DerivedB& rhs){
        static_assert(std::is_base_of_v<BoundExpression, DerivedA> && std::is_base_of_v<BoundExpression, DerivedB>, "BoundExpression::operator| template arguments need to be a BoundExpression or a derived class of Bound");
        BoundExpression expression(Relation::Union);

        if(std::is_same_v<DerivedA, BoundExpression> && lhs.relation_ == Relation::Union){
            std::copy(lhs.tree_.begin(), lhs.tree_.end(), std::back_inserter(expression.tree_));
        } else{
            expression.tree_.push_back(std::make_shared<const DerivedA>(lhs));
        }

        if(std::is_same_v<DerivedB, BoundExpression> && rhs.relation_ == Relation::Union){
            std::copy(rhs.tree_.begin(), rhs.tree_.end(), std::back_inserter(expression.tree_));
        } else{
            expression.tree_.push_back(std::make_shared<const DerivedB>(rhs));
        }

        return expression;
    }

    [[nodiscard]] virtual bool Contains(const VectorN& point) const {
        if(tree_.empty()){
            return true;
        }
        if(tree_.size() == 1){
            return tree_[0]->Contains(point);
        }
        return ContainerContains(tree_, point);
    }

    [[nodiscard]] virtual bool IsAtBoundary(const VectorN& point) const {
        if(tree_.empty()){
            return false;
        }
        if(tree_.size() == 1){
            return tree_[0]->IsAtBoundary(point);
        }

        // Classify each bound expression as whether the point is at its boundary
        std::unordered_set<BoundPtr> on_boundary;
        std::unordered_set<BoundPtr> not_on_boundary;
        for(const BoundPtr& ptr : tree_){
            if(ptr->IsAtBoundary(point)){
                on_boundary.insert(ptr);
            } else{
                not_on_boundary.insert(ptr);
            }
        }

        // If the point is not on any boundary, then it cannot be on the boundary of any finite boolean expression of bounds
        if(on_boundary.empty()){
            return false;
        }

        // For the bounds for which the point is not on its boundary:
        //   Union: those bounds should not contain the point
        //   Intersection: those bounds should all contain the point
        if(relation_ == Relation::Union){
            return !ContainerContains(not_on_boundary, point);
        } else {
            return ContainerContains(not_on_boundary, point);
        }
    }

    [[nodiscard]] virtual VectorN GetNearestPointWithinBound(const VectorN& point, const VectorN& prev_point) const {
        if(tree_.empty()){
            return point;
        }
        if(tree_.size() == 1){
            return tree_[0]->GetNearestPointWithinBound(point, prev_point);
        }
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
        });

        return prev_point + scale_opt * difference_vector;
    }

    [[nodiscard]] virtual std::vector<VectorN> GetSurfaceNormals(const VectorN& point) const {
        if(tree_.empty()){
            return std::vector<VectorN>();
        }
        if(tree_.size() == 1){
            return tree_[0]->GetSurfaceNormals(point);
        }

        // Find the bounds for which the point is at the boundary
        std::unordered_set<BoundPtr> on_boundary;
        std::copy_if(tree_.begin(), tree_.end(), std::inserter(on_boundary, on_boundary.begin()), [&point](const BoundPtr& ptr)->bool{
            return ptr->IsAtBoundary(point);
        });

        // If there are no bounds, then the point is not at a boundary
        if(on_boundary.empty()){
            return std::vector<VectorN>();
        }

        // Otherwise, return all the surface normal vectors
        std::vector<VectorN> total_surface_normals;
        for(const BoundPtr& ptr : on_boundary){
            const std::vector<VectorN> surface_normals = ptr->GetSurfaceNormals(point);
            total_surface_normals.insert(total_surface_normals.end(), surface_normals.begin(), surface_normals.end());
        }
        return total_surface_normals;
    } 

protected:
    Scalar BisectionSearch(std::function<bool(const Scalar&)> evaluator, const Scalar& tol = GTFO_ALGORITHMIC_CONVERGENCE_TOLERANCE) const {
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
    [[nodiscard]] bool ContainerContains(const Container& container, const VectorN& point) const {
        // any_of returns false if the container is empty
        // all_of returns true if the container is empty
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