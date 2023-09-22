//----------------------------------------------------------------------------------------------------
// File: NormBound.hpp
// Desc: Norm bound derived class
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "BoundBase.hpp"

namespace gtfo {

template <unsigned int Dimensions, typename Scalar = double>
class NormBound : public BoundBase<Dimensions, Scalar>{
public:
    using Base = BoundBase<Dimensions, Scalar>;
    using BoundPtr = std::shared_ptr<Base>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    NormBound(const Scalar &radius, const VectorN &center = VectorN::Zero(), const Scalar &tol = GTFO_EQUALITY_COMPARISON_TOLERANCE)
        : BoundBase<Dimensions, Scalar>(tol),
          radius_(radius),
          center_(center)
    {
        assert(radius >= 0.0);
    }

    [[nodiscard]] BoundPtr DeepCopy(void) const override {
        return std::make_shared<NormBound>(*this);
    }

    [[nodiscard]] bool Contains(const VectorN& point) const override {
        return (point - center_).norm() <= radius_;
    }

    [[nodiscard]] bool IsAtBoundary(const VectorN& point) const override {
        // Only tol distance inside the boundary is valid
        const Scalar point_shifted_origin_norm = (point - center_).norm();
        return (radius_ - this->tol_) <= point_shifted_origin_norm && point_shifted_origin_norm <= radius_;
    }

    [[nodiscard]] VectorN GetNearestPointWithinBound(const VectorN& point) const override {
        // If the point is already contained within the bound, then just return it
        if(Contains(point)){
            return point;
        }
        // Otherwise, clamp the norm to the radius
        const VectorN point_shifted_origin = point - center_;
        const Scalar clamped_norm = std::min(point_shifted_origin.norm(), radius_);
        return clamped_norm * point_shifted_origin.normalized() + center_;
    }

    [[nodiscard]] SurfaceNormals<VectorN> GetSurfaceNormals(const VectorN& point) const override {
        const VectorN point_shifted_origin = point - center_;

        // Special case where surface norms break down because our bound is smaller than our tolerance
        if (radius_ <= Base::tol_)
        {
            SurfaceNormals<VectorN> surface_normals;

            // For we spoof the surface normal to be a +/- unit vector in all dimensions to restrict movement
            for (unsigned i = 0; i < Dimensions; ++i)
            {
                surface_normals.push_back(VectorN::Unit(i));
                surface_normals.push_back(VectorN::Unit(i) * -1.0);
            }

            return surface_normals;
        }
        else if (point_shifted_origin.norm() < (radius_ - Base::tol_))
        {
            return SurfaceNormals<VectorN>();
        }

        // 2-norm: https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
        return SurfaceNormals<VectorN>(point_shifted_origin.normalized());
    }

private:
    const Scalar radius_;
    const VectorN center_;
};

}   // namespace gtfo