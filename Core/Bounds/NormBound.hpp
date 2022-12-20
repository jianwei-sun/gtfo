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

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    NormBound(const Scalar &radius, const VectorN &center = VectorN::Zero(), const Scalar &tol = GTFO_EQUALITY_COMPARISON_TOLERANCE)
        : BoundBase<Dimensions, Scalar>(tol),
          radius_(radius),
          center_(center)
    {
        assert(radius > 0.0);
    }

    [[nodiscard]] bool Contains(const VectorN& point) const override {
        return (point - center_).template lpNorm<Norm>() <= radius_;
    }

    [[nodiscard]] bool IsAtBoundary(const VectorN& point) const override {
        // Only tol distance inside the boundary is valid
        const Scalar point_shifted_origin_norm = (point - center_).template lpNorm<Norm>();
        return (radius_ - this->tol_) <= point_shifted_origin_norm && point_shifted_origin_norm <= radius_;
    }

    [[nodiscard]] VectorN GetNearestPointWithinBound(const VectorN& point, const VectorN& prev_point) const override {
        // An exact (and simple) solution exists in 1D
        if constexpr(Dimensions == 1){
            const VectorN point_shifted_origin = point - center_;
            return std::min(point_shifted_origin.template lpNorm<Norm>(), radius_) * point_shifted_origin.normalized() + center_;
        } 
        // Although exact solutions exist for specific cases, e.g. Norm = 2, Dimensions = 2, they are very complicated. It's easier to solve this numerically with the default implementation
        else {
            return BoundBase<Dimensions, Scalar>::GetNearestPointWithinBound(point, prev_point);
        }
    }

    [[nodiscard]] std::vector<VectorN> GetSurfaceNormals(const VectorN& point) const override {
        const VectorN point_shifted_origin = point - center_;
        // 2-norm: https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
        if constexpr(Norm == 2){
            return std::vector<VectorN>{point_shifted_origin.normalized()};
        }
        // p-norm (p >= 1): https://math.stackexchange.com/questions/1482494/derivative-of-the-l-p-norm
        else {
            const VectorN derivative = (point_shifted_origin.cwiseAbs() / point_shifted_origin.template lpNorm<Norm>()).array().pow(Norm - 1) * point_shifted_origin.array().sign();
            return std::vector<VectorN>{derivative.normalized()};
        }
    }

private:
    const Scalar radius_;
    const VectorN center_;
};

}   // namespace gtfo