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
#include <array>
#include <vector>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Utils/Containers.hpp"

namespace gtfo {

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

    class SurfaceNormalsCache{
    public:
        SurfaceNormalsCache() : point_(VectorN::Constant(NAN)) {}

        bool Contains(const VectorN& point) const{
            return (point.array() == point_.array()).all();
        }

        std::vector<VectorN> Get() const{
            return surface_normals_;
        }

        void Set(const VectorN& point, const std::vector<VectorN>& surface_normals){
            point_ = point;
            surface_normals_ = surface_normals;
        }

    private:
        VectorN point_;
        std::vector<VectorN> surface_normals_;
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
        return !GetSurfaceNormals(point).empty();
    }

    [[nodiscard]] virtual VectorN GetNearestPointWithinBound(const VectorN& point, const VectorN& prev_point) const {
        // If the currently point is already within the bound, then no additional work is needed
        if(Contains(point)){
            return point;
        }

        // Otherwise, bisection search until:
        //   Union: at least one expression in the set contains the point
        //   Intersection: all expressions in the set contain the point
        const VectorN difference_vector = point - prev_point;
        const Scalar scale_opt = BisectionSearch([&](const Scalar& scale)->bool{
            const VectorN test_point = prev_point + scale * difference_vector;
            return ContainerContains(tree_, test_point);
        });

        return prev_point + scale_opt * difference_vector;
    }

    [[nodiscard]] virtual std::vector<VectorN> GetSurfaceNormals(const VectorN& point) const {
        // Check the cache first since IsAtBoundary also calls GetSurfaceNormals, 
        // and the two are usually used together
        if(surface_normals_cache_.Contains(point)){
            return surface_normals_cache_.Get();
        }
        if(tree_.empty()){
            return std::vector<VectorN>();
        }
        if(tree_.size() == 1){
            return tree_[0]->GetSurfaceNormals(point);
        }

        // Classify each bound expression as whether the point is at its boundary
        std::vector<BoundPtr> bounds_with_point_on_boundary;
        std::vector<BoundPtr> bounds_with_point_not_on_boundary;
        for(const BoundPtr& ptr : tree_){
            if(ptr->IsAtBoundary(point)){
                bounds_with_point_on_boundary.push_back(ptr);
            } else{
                bounds_with_point_not_on_boundary.push_back(ptr);
            }
        }

        // If the query point is not on any boundary, or if it is an interior point, 
        // or an exterior point, then there can be no active bounds
        if(bounds_with_point_on_boundary.empty() || 
            (relation_ == Relation::Union && ContainerContains(bounds_with_point_not_on_boundary, point)) ||
            (relation_ == Relation::Intersection && !ContainerContains(bounds_with_point_not_on_boundary, point)))
        {
            return std::vector<VectorN>();
        }

        // Construct the vector of surface normals
        std::vector<VectorN> combined_surface_normals;

        // Union case: annihilate surface normals that point exactly opposite of each other; these occur when boundaries 
        // are merged together. For example, if the bound expression is a union of two unit rectangles placed tangent to 
        // each other along a side, then that side should not be a boundary
        if(relation_ == Relation::Union){
            for(const BoundPtr& ptr: bounds_with_point_on_boundary){
                for(const VectorN& surface_normal : ptr->GetSurfaceNormals(point)){
                    // Check if there already is a surface normal that points in the opposite direction
                    const typename std::vector<VectorN>::iterator it = std::find_if(combined_surface_normals.begin(), combined_surface_normals.end(), [&surface_normal](const VectorN& vector)->bool{
                        return IsEqual(-surface_normal, vector);
                    });
                    // If there is, remove it
                    if(it != combined_surface_normals.end()){
                        combined_surface_normals.erase(it);
                    // Otherwise, insert the surface normal to the collection
                    } else{
                        combined_surface_normals.push_back(surface_normal);
                    }
                }
            }
        }
        // Intersection case: simply take all the surface normals
        else {
            for(const BoundPtr& ptr : bounds_with_point_on_boundary){
                const std::vector<VectorN> surface_normals = ptr->GetSurfaceNormals(point);
                combined_surface_normals.insert(combined_surface_normals.end(), surface_normals.begin(), surface_normals.end());
            }
        }

        // Remove exact duplicates. Uses a vector instead of an unsorted_set because the
        // number of elements is small
        std::vector<VectorN> duplicates;
        const auto new_end = std::remove_if(combined_surface_normals.begin(), combined_surface_normals.end(), [&duplicates](const VectorN& vector)->bool{
            if(std::find(duplicates.begin(), duplicates.end(), vector) != duplicates.end()){
                return true;
            } else{
                duplicates.push_back(vector);
                return false;
            }
        });
        combined_surface_normals.erase(new_end, combined_surface_normals.end());

        // Update the cache and return the result
        surface_normals_cache_.Set(point, combined_surface_normals);
        return combined_surface_normals;
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
    mutable SurfaceNormalsCache surface_normals_cache_;
};

}   // namespace gtfo