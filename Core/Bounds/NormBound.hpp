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

    NormBound(const Scalar& threshold, const VectorN& center = VectorN::Zero())
        :   threshold_(threshold),
            center_(center)
    {
        assert(threshold > 0.0);
    }

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    // The constant Eigen::Infinity, defined as -1, can be used for the infinite norm
    bool Contains(const VectorN& point) const override {
        return (point - center_).template lpNorm<Norm>() <= threshold_;
    }

    VectorN operator()(const VectorN& point, const VectorN& prev_point, const Scalar& tol = 0.01) const override {
        // An exact (and simple) solution exists in 1D
        if constexpr(Dimensions == 1){
            const VectorN point_shifted_origin = point - center_;
            return std::min(point_shifted_origin.template lpNorm<Norm>(), threshold_) * point_shifted_origin.normalized() + center_;
        } 
        // Although exact solutions exist for specific cases, e.g. Norm = 2, Dimensions = 2, they are very complicated. It's easier to solve this numerically with the default implementation
        else {
            return BoundBase<Dimensions, Scalar>::operator()(point, prev_point, tol);
        }
    }

private:
    const Scalar threshold_;
    const VectorN center_;
};

}   // namespace gtfo