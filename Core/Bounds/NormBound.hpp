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

    NormBound(const Scalar& threshold, const VectorN& center = VectorN::Zero(), const Scalar& tol = GTFO_EQUALITY_COMPARISON_TOLERANCE)
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

    // TODO: Possibly deprecate the nondifferentiable norms (1 and infinite) and use RectangleBound instead
    [[nodiscard]] std::vector<VectorN> GetSurfaceNormals(const VectorN& point) const override {
        const VectorN point_shifted_origin = point - center_;
        // 1-norm: 
        if constexpr(Norm == 1){
            // Determine if any coordinate is at a corner, since there are more normal vectors there
            Eigen::Matrix<bool, Dimensions, 1> at_corner = (point_shifted_origin.array().abs() - threshold_).abs() <= this->tol_;

            // If the point is not at any corner, then there is only one normal vector according to:
            // https://math.stackexchange.com/questions/1395699/differentiation-of-1-norm-of-a-vector
            if(!at_corner.any() || Dimensions == 1){
                return std::vector<VectorN>{VectorN(point_shifted_origin.array().sign()).normalized()};
            } 
            // Otherwise, there are 2 * (Dimensions - 1) normal vectors
            else{
                // Ensure that only one coordinate is at the corner
                assert(at_corner.template cast<unsigned int>().sum() == 1);

                // Get the coordinate of the corner
                size_t index_of_corner;
                at_corner.maxCoeff(&index_of_corner);
                const VectorN corner_coordinate = -VectorN::Unit(index_of_corner).array() * point_shifted_origin.array().sign();

                // Insert the normal vectors
                std::vector<VectorN> surface_normals;
                for(size_t i = 0; i < Dimensions; ++i){
                    if(i != index_of_corner){
                        surface_normals.push_back((corner_coordinate + VectorN::Unit(i)).normalized());
                        surface_normals.push_back((corner_coordinate - VectorN::Unit(i)).normalized());
                    }
                }
                return surface_normals;
            }            
        } 
        // 2-norm: https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
        else if constexpr(Norm == 2){
            return std::vector<VectorN>{point_shifted_origin.normalized()};
        }
        // Infinity-norm: https://math.stackexchange.com/questions/2696519/finding-the-derivative-of-the-infinity-norm
        else if constexpr(Norm == Eigen::Infinity){
            // const VectorN derivative = ((point_shifted_origin.cwiseAbs().array() - point_shifted_origin.cwiseAbs().maxCoeff()).cwiseAbs() < this->tol_).template cast<Scalar>() * point_shifted_origin.array().sign();
            // return derivative.normalized();
            std::vector<VectorN> surface_normals;
            const VectorN signs = point_shifted_origin.array().sign();
            const Eigen::Matrix<bool, Dimensions, 1> selected = (point_shifted_origin.cwiseAbs().array() - point_shifted_origin.cwiseAbs().maxCoeff()).cwiseAbs() < this->tol_;
            for(size_t i = 0; i < Dimensions; ++i){
                if(selected(i)){
                    surface_normals.push_back(VectorN::Unit(i) * signs(i));
                }
            }
            return surface_normals;
        }
        // p-norm (p >= 1): https://math.stackexchange.com/questions/1482494/derivative-of-the-l-p-norm
        else {
            const VectorN derivative = (point_shifted_origin.cwiseAbs() / point_shifted_origin.template lpNorm<Norm>()).array().pow(Norm - 1) * point_shifted_origin.array().sign();
            return std::vector<VectorN>{derivative.normalized()};
        }
    }

private:
    const Scalar threshold_;
    const VectorN center_;
};

}   // namespace gtfo