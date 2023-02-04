//----------------------------------------------------------------------------------------------------
// File: NormBound.hpp
// Desc: Norm bound derived class
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "BoundBase.hpp"

namespace gtfo {

template <unsigned int Norm, unsigned int Dimensions, typename Scalar = double>
class NormBound : public BoundBase<Dimensions, Scalar>{
public:
    static_assert(Norm >= 2, "Norm argument needs to be at least 2");
    using Base = BoundBase<Dimensions, Scalar>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    NormBound(const Scalar &radius, const VectorN &center = VectorN::Zero(), const Scalar &tol = GTFO_EQUALITY_COMPARISON_TOLERANCE)
        : BoundBase<Dimensions, Scalar>(tol),
          radius_(radius),
          center_(center)
    {
        assert(radius >= 0.0);
    }

    [[nodiscard]] bool Contains(const VectorN& point) const override {
        return (point - center_).template lpNorm<Norm>() <= radius_;
    }

    [[nodiscard]] bool IsAtBoundary(const VectorN& point) const override {
        // Only tol distance inside the boundary is valid
        const Scalar point_shifted_origin_norm = (point - center_).template lpNorm<Norm>();
        return (radius_ - Base::tol_) <= point_shifted_origin_norm && point_shifted_origin_norm <= radius_;
    }

    [[nodiscard]] VectorN GetNearestPointWithinBound(const VectorN& point) const override {
        // If the point is already contained within the bound, then just return it
        if(Contains(point)){
            return point;
        }
        // Otherwise, clamp the norm to the radius
        const VectorN point_shifted_origin = point - center_;
        const Scalar clamped_norm = std::min(point_shifted_origin.template lpNorm<Norm>(), radius_);
        if constexpr(Norm == 2){
            return clamped_norm * point_shifted_origin.normalized() + center_;
        } else{
            return clamped_norm / point_shifted_origin.template lpNorm<Norm>() * point_shifted_origin + center_;
        }
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
        else
        {
            if (point_shifted_origin.template lpNorm<Norm>() < (radius_ - Base::tol_))
            {
                return SurfaceNormals<VectorN>();
            }
        }

        // 2-norm: https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
        if constexpr(Norm == 2){
            return SurfaceNormals<VectorN>(point_shifted_origin.normalized());
        }
        // p-norm (p >= 1): https://math.stackexchange.com/questions/1482494/derivative-of-the-l-p-norm
        else {
            const VectorN derivative = (point_shifted_origin.cwiseAbs() / point_shifted_origin.template lpNorm<Norm>()).array().pow(Norm - 1) * point_shifted_origin.array().sign();
            return SurfaceNormals<VectorN>(derivative.normalized());
        }
    }

private:
    const Scalar radius_;
    const VectorN center_;
};

}   // namespace gtfo