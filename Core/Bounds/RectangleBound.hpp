//----------------------------------------------------------------------------------------------------
// File: RectangleBound.hpp
// Desc: Rectangle bound derived class
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "BoundBase.hpp"

namespace gtfo
{

    template <unsigned int Dimensions, typename Scalar = double>
    class RectangleBound : public BoundBase<Dimensions, Scalar>
    {
    public:
        using Base = BoundBase<Dimensions, Scalar>;
        using BoundPtr = std::shared_ptr<Base>;
        using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

        RectangleBound(const VectorN &lower_limits, const VectorN &upper_limits, const VectorN &center, const Scalar& tol = GTFO_EQUALITY_COMPARISON_TOLERANCE)
            : BoundBase<Dimensions, Scalar>(tol),
              lower_limits_(lower_limits),
              upper_limits_(upper_limits),
              center_(center)
        {
            // All lower limits must be lower than their respective upper limits
            assert((lower_limits.array() <= upper_limits.array()).all());
        }

        // Case where we want same limit ditance in upper and lower bounds
        RectangleBound(const VectorN &bilateral_limits, const VectorN& center = VectorN::Zero(), const Scalar& tol = GTFO_EQUALITY_COMPARISON_TOLERANCE)
            : BoundBase<Dimensions, Scalar>(tol),
              lower_limits_(-bilateral_limits),
              upper_limits_(bilateral_limits),
              center_(center)
        {
            // All lower limits must be lower than their respective upper limits
            assert((bilateral_limits.array() >= 0.0).all());
        }

        [[nodiscard]] BoundPtr DeepCopy(void) const override {
            return std::make_shared<RectangleBound>(*this);
        }

        [[nodiscard]] bool Contains(const VectorN &point) const override
        {
            // If all upper and lower boundaries are satisfied we are inside
            return ((point.array() >= (center_ + lower_limits_).array()).all() &&
                    (point.array() <= (center_ + upper_limits_).array()).all());
        }

        [[nodiscard]] bool IsAtBoundary(const VectorN &point) const override
        {
            // Any inside position within tolerance of a boundary means our point is at a boundary
            const Eigen::Array<Scalar, Dimensions, 1> point_shifted_origin = (point - center_).array();
            return ((lower_limits_.array() <= point_shifted_origin && point_shifted_origin <= (lower_limits_.array() + Base::tol_).array()).any() 
                   || ((upper_limits_.array() - Base::tol_).array() <= point_shifted_origin && point_shifted_origin <= upper_limits_.array()).any()) 
                   && Contains(point);
        }

        [[nodiscard]] VectorN GetNearestPointWithinBound(const VectorN &point) const override
        {
            const VectorN point_shifted_origin = point - center_;
            return point_shifted_origin.cwiseMax(lower_limits_).cwiseMin(upper_limits_) + center_;
        }

        [[nodiscard]] SurfaceNormals<VectorN> GetSurfaceNormals(const VectorN &point) const override
        {
            const VectorN point_shifted_origin = point - center_;

            // Determine if the point is at a limit. Note that the point can be at both the upper and lower limit, if they are the same (within tolerance)
            const Eigen::Matrix<bool, Dimensions, 1> at_or_above_upper_limit = (point_shifted_origin.array() >= (upper_limits_.array() - Base::tol_));
            const Eigen::Matrix<bool, Dimensions, 1> at_or_below_lower_limit = (point_shifted_origin.array() <= (lower_limits_.array() + Base::tol_));

            SurfaceNormals<VectorN> surface_normals;

            // For each coordinate that is at a limit, the corresponding surface normal is a +/- unit vector
            for (unsigned i = 0; i < Dimensions; ++i)
            {
                if(at_or_above_upper_limit(i)){
                    surface_normals.push_back(VectorN::Unit(i));
                }
                if(at_or_below_lower_limit(i)){
                    surface_normals.push_back(VectorN::Unit(i) * -1.0);
                }
            }
            return surface_normals;
        }

    private:
        const VectorN lower_limits_; // Relative coordinates from the center to the minimum corner point of the N dimensional rectangle
        const VectorN upper_limits_; // Relative coordinates from the center the maximum corner point of the N dimensional rectangle
        const VectorN center_;       // Absolute Coordinates defining the center point of the rectangle bound (note this does not have to be the geometric center)
    };

} // namespace gtfo