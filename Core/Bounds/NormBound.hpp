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

    NormBound(const Scalar& threshold)
        :   threshold_(threshold)
    {
        assert(threshold > 0.0);
    }

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    // The constant Eigen::Infinity, defined as -1, can be used for the infinite norm
    bool Contains(const VectorN& point) const override {
        return point.template lpNorm<Norm>() <= threshold_;
    }

    VectorN operator()(const VectorN& point) const override {
        const Scalar norm = point.template lpNorm<Norm>();
        return std::min(norm, threshold_) * point;
    }

private:
    const Scalar threshold_;
};

}   // namespace gtfo