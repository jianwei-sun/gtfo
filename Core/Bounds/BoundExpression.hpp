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
    }

    [[nodiscard]] virtual bool IsAtBoundary(const VectorN& point) const {
        return !GetSurfaceNormals(point).IsEmpty();
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
        // Surface normals are empty if the point is not contained
        if(!Contains(point)){
            return SurfaceNormals();
        }

        const std::function<SurfaceNormals(const BoundBase&)> get_bound_surface_normals = 
            [&point](const BoundBase& bound)->SurfaceNormals{
                return bound.GetSurfaceNormals(point);
            };

        const std::function<SurfaceNormals(const Relation&, const std::vector<SurfaceNormals>&)> combine_surface_normals = 
            [](const Relation& relation, const std::vector<SurfaceNormals>& collection)->SurfaceNormals{
                if(relation == Relation::Intersection){
                    return SurfaceNormals(Relation::Union, collection);
                } else{
                    return SurfaceNormals(Relation::Intersection, collection);
                }
            };

        return MapReduce(get_bound_surface_normals, combine_surface_normals);
    } 
};

}   // namespace gtfo