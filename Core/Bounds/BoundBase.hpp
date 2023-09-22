//----------------------------------------------------------------------------------------------------
// File: BoundBase.hpp
// Desc: base class for bounds
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <memory>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Utils/Constants.hpp"
#include "SurfaceNormals.hpp"

namespace gtfo {

template <unsigned int Dimensions, typename Scalar = double>
class BoundBase {
public:
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using BoundPtr = std::shared_ptr<BoundBase>;

    BoundBase(const Scalar& tol = GTFO_EQUALITY_COMPARISON_TOLERANCE) : tol_(tol) {}
    BoundBase(const BoundBase& other) = default;

    // Deep copy functionality
    [[nodiscard]] virtual BoundPtr DeepCopy(void) const {
        return std::make_shared<BoundBase>(*this);
    }

    // Returns whether the given point is contained in the bound
    [[nodiscard]] virtual bool Contains(const VectorN& point) const {
        return true;
    }

    // Returns whether the given point is on the boundary of the bound to the internal tolerance tol_
    [[nodiscard]] virtual bool IsAtBoundary(const VectorN& point) const {
        return false;
    }

    // Finds the nearest point contained within the convex bound to the given point
    [[nodiscard]] virtual VectorN GetNearestPointWithinBound(const VectorN& point) const {
        // Every point is within the bound for this base class
        return point;
    }

    // Returns a collection of surface unit normal vectors at the given point. For smooth shapes, this
    // should almost always return a single unit normal. However, for bounds with corners (such as rectangles)
    // multiple surface normals are needed to correctly update the dynamics at the bound.
    [[nodiscard]] virtual SurfaceNormals<VectorN> GetSurfaceNormals(const VectorN& point) const {
        // Since there are no boundaries in this base class, return an empty surface normals collection
        return SurfaceNormals<VectorN>();
    }

protected:
    const Scalar tol_;
};

}   // namespace gtfo