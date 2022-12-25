//----------------------------------------------------------------------------------------------------
// File: BoundExpression.hpp
// Desc: base class for boolean expressions of bounds
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <algorithm>
#include <iterator>
#include <memory>
#include <unordered_set>
#include <array>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Utils/BooleanExpression.hpp"
#include "../Utils/Containers.hpp"
#include "BoundBase.hpp"
#include "SurfaceNormals.hpp"

namespace gtfo {


template <unsigned int Dimensions, typename Scalar>
class BoundExpression : public BooleanExpression<BoundBase<Dimensions, Scalar>>{

    static_assert(Dimensions > 0, "Dimensions must be at least 1.");
    static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");

    using BoundBase = BoundBase<Dimensions, Scalar>;
    using BooleanExpression = BooleanExpression<BoundBase>;

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using SurfaceNormals = SurfaceNormals<VectorN>;

    // class SurfaceNormalsCache{
    // public:
    //     SurfaceNormalsCache() : point_(VectorN::Constant(NAN)) {}

    //     bool Contains(const VectorN& point) const{
    //         return (point.array() == point_.array()).all();
    //     }

    //     std::vector<VectorN> Get() const{
    //         return surface_normals_;
    //     }

    //     void Set(const VectorN& point, const std::vector<VectorN>& surface_normals){
    //         point_ = point;
    //         surface_normals_ = surface_normals;
    //     }

    // private:
    //     VectorN point_;
    //     std::vector<VectorN> surface_normals_;
    // };

public:
    BoundExpression() : BooleanExpression(Relation::Intersection) {}
    BoundExpression(const Relation& relation) : BooleanExpression(relation) {}
    BoundExpression(const BooleanExpression& expression) : BooleanExpression(expression) {}

    template <typename... InputTypes>
    BoundExpression(const Relation& relation, const InputTypes&... inputs) 
        : BooleanExpression(relation, inputs...) {}



    template <typename LeftType, typename RightType>
    [[nodiscard]] friend typename std::enable_if_t<
        // Expression & Expression
        (std::is_same_v<BoundExpression, LeftType> && std::is_same_v<BoundExpression, RightType>) || 
        // Expression & Bound
        (std::is_same_v<BoundExpression, LeftType> && std::is_base_of_v<BoundBase, RightType>) ||
        // Bound & Expression
        (std::is_base_of_v<BoundBase, LeftType> && std::is_same_v<BoundExpression, RightType>),
    BoundExpression> 
    operator&(const LeftType& lhs, const RightType& rhs){
        return BoundExpression(Relation::Intersection, lhs, rhs);
    }

    template <typename LeftType, typename RightType>
    [[nodiscard]] friend typename std::enable_if_t<
        // Expression | Expression
        (std::is_same_v<BoundExpression, LeftType> && std::is_same_v<BoundExpression, RightType>) ||
        // Expression | Bound
        (std::is_same_v<BoundExpression, LeftType> && std::is_base_of_v<BoundBase, RightType>) ||
        // Bound | Expression
        (std::is_base_of_v<BoundBase, LeftType> && std::is_same_v<BoundExpression, RightType>),
    BoundExpression> 
    operator|(const LeftType& lhs, const RightType& rhs){
        return BoundExpression(Relation::Union, lhs, rhs);
    }

    [[nodiscard]] virtual bool Contains(const VectorN& point) const {
        const std::function<bool(const BoundBase&)> bound_contains = 
            [&](const BoundBase& bound)->bool{
                return bound.Contains(point);
            };

        const std::function<bool(const Relation&, const std::vector<bool>&)> combine_results = 
            [&](const Relation& relation, const std::vector<bool>& results)->bool{
                if(relation == Relation::Union){
                    // True if any is true
                    return std::find(results.cbegin(), results.cend(), true) != results.cend();
                } else{
                    // True if all is true
                    return std::find(results.cbegin(), results.cend(), false) == results.cend();
                }
            };

        return MapReduce(bound_contains, combine_results);



        // if(tree_.empty()){
        //     return true;
        // }
        // if(tree_.size() == 1){
        //     return tree_[0]->Contains(point);
        // }
        // return ContainerContains(tree_, point);
    }

    [[nodiscard]] virtual bool IsAtBoundary(const VectorN& point) const {
        using BoundPtr = std::shared_ptr<const BoundBase>;
        using BoundsWithResult = std::pair<std::vector<BoundPtr>, bool>;
        // if(tree_.empty()){
        //     return false;
        // }
        // if(tree_.size() == 1){
        //     return tree_[0]->IsAtBoundary(point);
        // }
        // return !GetSurfaceNormals(point).empty();
        const std::function<BoundsWithResult(const BoundBase&)> point_at_bound_boundary = 
            [&point](const BoundBase& bound)->BoundsWithResult{
                return std::make_pair(std::vector<BoundPtr>{BoundPtr(&bound)}, bound.IsAtBoundary(point));
            };

        const std::function<BoundsWithResult(const Relation&, const std::vector<BoundsWithResult>&)> combine_results = 
            [&point](const Relation& relation, const std::vector<BoundsWithResult>& collection)->BoundsWithResult{
                std::vector<BoundPtr> not_on_boundary;
                for(const BoundsWithResult& bounds_with_result : collection){
                    if(!bounds_with_result.second){
                        std::copy(bounds_with_result.first.cbegin(), bounds_with_result.first.cend(), std::back_inserter(not_on_boundary));
                    }
                }

                // For the bounds for which the point is not on its boundary:
                //   Union: those bounds should not contain the point
                //   Intersection: those bounds should all contain the point
                if(relation == Relation::Union){
                    return std::make_pair(not_on_boundary, !std::any_of(not_on_boundary.cbegin(), not_on_boundary.cend(), 
                        [&point](const BoundPtr& ptr)->bool{
                            return ptr->Contains(point);
                        }
                    ));
                } else {
                    return std::make_pair(not_on_boundary, std::all_of(not_on_boundary.cbegin(), not_on_boundary.cend(), 
                        [&point](const BoundPtr& ptr)->bool{
                            return ptr->Contains(point);
                        }
                    ));
                }
            };

        return MapReduce(point_at_bound_boundary, combine_results).second;
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
        const Scalar scale_opt = BisectionSearch<Scalar>([&](const Scalar& scale)->bool{
            const VectorN test_point = prev_point + scale * difference_vector;
            return Contains(test_point);
        });

        return prev_point + scale_opt * difference_vector;
    }

    [[nodiscard]] virtual SurfaceNormals GetSurfaceNormals(const VectorN& point) const {
        // return std::vector<VectorN>();

        const std::function<SurfaceNormals(const BoundBase&)> get_bound_surface_normals = 
            [&point](const BoundBase& bound)->SurfaceNormals{
                return bound.GetSurfaceNormals(point);
            };

        const std::function<SurfaceNormals(const Relation&, const std::vector<SurfaceNormals>&)> combine_surface_normals = 
            [](const Relation& relation, const std::vector<SurfaceNormals>& collection)->SurfaceNormals{
                return SurfaceNormals(relation, collection);
            };

        return MapReduce(get_bound_surface_normals, combine_surface_normals);

        // // Check the cache first since IsAtBoundary also calls GetSurfaceNormals, 
        // // and the two are usually used together
        // if(surface_normals_cache_.Contains(point)){
        //     return surface_normals_cache_.Get();
        // }
        // if(tree_.empty()){
        //     return std::vector<VectorN>();
        // }
        // if(tree_.size() == 1){
        //     return tree_[0]->GetSurfaceNormals(point);
        // }

        // // Classify each bound expression as whether the point is at its boundary
        // std::vector<BoundPtr> bounds_with_point_on_boundary;
        // std::vector<BoundPtr> bounds_with_point_not_on_boundary;
        // for(const BoundPtr& ptr : tree_){
        //     if(ptr->IsAtBoundary(point)){
        //         bounds_with_point_on_boundary.push_back(ptr);
        //     } else{
        //         bounds_with_point_not_on_boundary.push_back(ptr);
        //     }
        // }

        // // If the query point is not on any boundary, or if it is an interior point, 
        // // or an exterior point, then there can be no active bounds
        // if(bounds_with_point_on_boundary.empty() || 
        //     (relation_ == Relation::Union && ContainerContains(bounds_with_point_not_on_boundary, point)) ||
        //     (relation_ == Relation::Intersection && !ContainerContains(bounds_with_point_not_on_boundary, point)))
        // {
        //     return std::vector<VectorN>();
        // }

        // // Construct the vector of surface normals
        // std::vector<VectorN> combined_surface_normals;

        // // Union case: annihilate surface normals that point exactly opposite of each other; these occur when boundaries 
        // // are merged together. For example, if the bound expression is a union of two unit rectangles placed tangent to 
        // // each other along a side, then that side should not be a boundary
        // if(relation_ == Relation::Union){
        //     for(const BoundPtr& ptr: bounds_with_point_on_boundary){
        //         for(const VectorN& surface_normal : ptr->GetSurfaceNormals(point)){
        //             // Check if there already is a surface normal that points in the opposite direction
        //             const typename std::vector<VectorN>::iterator it = std::find_if(combined_surface_normals.begin(), combined_surface_normals.end(), [&surface_normal](const VectorN& vector)->bool{
        //                 return IsEqual(-surface_normal, vector);
        //             });
        //             // If there is, remove it
        //             if(it != combined_surface_normals.end()){
        //                 combined_surface_normals.erase(it);
        //             // Otherwise, insert the surface normal to the collection
        //             } else{
        //                 combined_surface_normals.push_back(surface_normal);
        //             }
        //         }
        //     }
        // }
        // // Intersection case: simply take all the surface normals
        // else {
        //     for(const BoundPtr& ptr : bounds_with_point_on_boundary){
        //         const std::vector<VectorN> surface_normals = ptr->GetSurfaceNormals(point);
        //         combined_surface_normals.insert(combined_surface_normals.end(), surface_normals.begin(), surface_normals.end());
        //     }
        // }

        // // Remove exact duplicates. Uses a vector instead of an unsorted_set because the
        // // number of elements is small
        // std::vector<VectorN> duplicates;
        // const auto new_end = std::remove_if(combined_surface_normals.begin(), combined_surface_normals.end(), [&duplicates](const VectorN& vector)->bool{
        //     if(std::find(duplicates.begin(), duplicates.end(), vector) != duplicates.end()){
        //         return true;
        //     } else{
        //         duplicates.push_back(vector);
        //         return false;
        //     }
        // });
        // combined_surface_normals.erase(new_end, combined_surface_normals.end());

        // // Update the cache and return the result
        // surface_normals_cache_.Set(point, combined_surface_normals);
        // return combined_surface_normals;
    } 

protected:
    // Scalar BisectionSearch(std::function<bool(const Scalar&)> evaluator, const Scalar& tol = GTFO_ALGORITHMIC_CONVERGENCE_TOLERANCE) const {
    //     std::pair<Scalar, Scalar> search_interval = std::make_pair(0.0, 1.0);
    //     while((search_interval.second - search_interval.first) > tol){
    //         const Scalar scale = (search_interval.first + search_interval.second) / 2.0;
    //         if(evaluator(scale)){
    //             search_interval.first = scale;
    //         } else{
    //             search_interval.second = scale;
    //         }
    //     }
    //     return search_interval.first;
    // }

private:
    // template <typename Container>
    // [[nodiscard]] bool ContainerContains(const Container& container, const VectorN& point) const {
    //     // any_of returns false if the container is empty
    //     // all_of returns true if the container is empty
    //     if(relation_ == Relation::Union){
    //         return std::any_of(container.begin(), container.end(), [&point](const BoundPtr& ptr)->bool{
    //             return ptr->Contains(point);
    //         });
    //     } else{ // Relation::Intersection
    //         return std::all_of(container.begin(), container.end(), [&point](const BoundPtr& ptr)->bool{
    //             return ptr->Contains(point);
    //         });
    //     }
    // }

    // Relation relation_;
    // std::vector<BoundPtr> tree_;
    // mutable SurfaceNormalsCache surface_normals_cache_;
};


}   // namespace gtfo