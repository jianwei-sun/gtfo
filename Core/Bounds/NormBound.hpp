//----------------------------------------------------------------------------------------------------
// File: NormBound.hpp
// Desc: Norm bound derived class
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "BoundBase.hpp"

namespace gtfo {

template <int Norm, unsigned int Dimensions, typename Scalar = double>
class NormBound : public BoundBase<Dimensions, Scalar>{
public:
    static_assert(Norm >= 1 || Norm == -1, "Norm argument needs to be at least 1 or be -1");

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    NormBound(const Scalar& threshold, const VectorN& center = VectorN::Zero(), const Scalar& tol = EQUALITY_COMPARISON_TOLERANCE)
        :   BoundBase<Dimensions, Scalar>(tol),
            threshold_(threshold),
            center_(center)
    {
        assert(threshold > 0.0);
    }

    // The constant Eigen::Infinity, defined as -1, can be used for the infinite norm
    [[nodiscard]] bool Contains(const VectorN& point) const override {
        return (point - center_).template lpNorm<Norm>() <= threshold_;
    }

    [[nodiscard]] bool IsAtBoundary(const VectorN& point) const override {
        return std::abs((point - center_).template lpNorm<Norm>() - threshold_) <= this->tol_;
    }

    [[nodiscard]] VectorN GetNearestPointWithinBound(const VectorN& point, const VectorN& prev_point) const override {
        // An exact (and simple) solution exists in 1D
        if constexpr(Dimensions == 1){
            const VectorN point_shifted_origin = point - center_;
            return std::min(point_shifted_origin.template lpNorm<Norm>(), threshold_) * point_shifted_origin.normalized() + center_;
        } 
        // Although exact solutions exist for specific cases, e.g. Norm = 2, Dimensions = 2, they are very complicated. It's easier to solve this numerically with the default implementation
        else {
            return BoundBase<Dimensions, Scalar>::GetNearestPointWithinBound(point, prev_point);
        }
    }

    [[nodiscard]] VectorN GetSurfaceNormal(const VectorN& point) const override {
        const VectorN point_shifted_origin = point - center_;
        // 1-norm: https://math.stackexchange.com/questions/1395699/differentiation-of-1-norm-of-a-vector
        if constexpr(Norm == 1){
            const VectorN derivative = point_shifted_origin.array().sign();
            return derivative.normalized();
        } 
        // 2-norm: https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
        else if constexpr(Norm == 2){
            return point_shifted_origin.normalized();
        }
        // Infinity-norm: https://math.stackexchange.com/questions/2696519/finding-the-derivative-of-the-infinity-norm
        else if constexpr(Norm == Eigen::Infinity){
            const VectorN derivative = ((point_shifted_origin.cwiseAbs().array() - point_shifted_origin.cwiseAbs().maxCoeff()).cwiseAbs() < this->tol_).template cast<Scalar>() * point_shifted_origin.array().sign();
            return derivative.normalized();
        }
        // p-norm (p >= 1): https://math.stackexchange.com/questions/1482494/derivative-of-the-l-p-norm
        else {
            const VectorN derivative = (point_shifted_origin.cwiseAbs() / point_shifted_origin.template lpNorm<Norm>()).array().pow(Norm - 1) * point_shifted_origin.array().sign();
            return derivative.normalized();
        }
    }

private:
    const Scalar threshold_;
    const VectorN center_;
};

}   // namespace gtfo