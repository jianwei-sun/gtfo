//----------------------------------------------------------------------------------------------------
// File: BoundBase.hpp
// Desc: base class for bounds
//----------------------------------------------------------------------------------------------------
#pragma once

// Third-party dependencies
#include <Eigen/Dense>
#include <memory>

// Project-specific
#include "../Utils/Algorithms.hpp"
#include "../Utils/BooleanExpression.hpp"
#include "../Utils/Constants.hpp"

namespace gtfo {

template <unsigned int Dimensions, typename Scalar>
class BoundExpression;

template <unsigned int Dimensions, typename Scalar = double>
class BoundBase {
public:
    using BoundExpression = BoundExpression<Dimensions, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    BoundBase(const Scalar& tol = GTFO_EQUALITY_COMPARISON_TOLERANCE) : tol_(tol) {}
    BoundBase(const BoundBase& other) = default;

    // Intersection of two bounds should return a BoundExpression with Intersection
    template <typename LeftType, typename RightType>
    [[nodiscard]] friend typename std::enable_if_t<
        std::is_base_of_v<BoundBase, LeftType> && 
        std::is_base_of_v<BoundBase, RightType>, 
    BoundExpression> 
    operator&(const LeftType& lhs, const RightType& rhs){
        return BoundExpression(Relation::Intersection, lhs, rhs);
    }

    // Union of two bounds should return a BoundExpression with Union
    template <typename LeftType, typename RightType>
    [[nodiscard]] friend typename std::enable_if_t<
        std::is_base_of_v<BoundBase, LeftType> && 
        std::is_base_of_v<BoundBase, RightType>, 
    BoundExpression> 
    operator|(const LeftType& lhs, const RightType& rhs){
        return BoundExpression(Relation::Union, lhs, rhs);
    }

    // Returns whether the given point is contained in the bound
    [[nodiscard]] virtual bool Contains(const VectorN& point) const {
        return true;
    }

    // Returns whether the given point is on the boundary of the bound to the internal tolerance tol_
    [[nodiscard]] virtual bool IsAtBoundary(const VectorN& point) const {
        return false;
    }

    // Finds the nearest point contained within the bound to the given point, along the line joining prev_point
    // and point. This is necessary for non-convex bounds, which do not have a unique closest point. 
    [[nodiscard]] virtual VectorN GetNearestPointWithinBound(const VectorN& point, const VectorN& prev_point) const {
        // Assume that the previous point is within the bounds
        assert(Contains(prev_point));

        // Return if there is no bound violation
        if(Contains(point)){
            return point;
        }

        // Otherwise bisection search
        const VectorN difference_vector = point - prev_point;
        const Scalar scale_opt = BisectionSearch<Scalar>([&](const Scalar& scale){
            const VectorN test_point = prev_point + scale * difference_vector;
            return Contains(test_point);
        });
        return prev_point + scale_opt * difference_vector;
    }

    // Returns a vector of outward-pointing surface unit vectors at the given point. For smooth shapes, this
    // should almost always return a single unit normal. However, for bounds with corners (such as rectangles)
    // multiple surface normals are needed to correctly update the dynamics at the bound. If the query point is
    // not on the boundary, then just return an empty vector
    [[nodiscard]] virtual std::vector<VectorN> GetSurfaceNormals(const VectorN& point) const {
        return std::vector<VectorN>();
    }

protected:
    const Scalar tol_;
};

}   // namespace gtfo