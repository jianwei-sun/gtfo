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
        using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

        RectangleBound(const VectorN &lower_limits, const VectorN &upper_limits, const VectorN &center = VectorN::Zero())
            : lower_limits_(lower_limits),
              upper_limits_(upper_limits),
              center_(center)
        {
            // All lower limits must be lower than their respective upper limits
            for (unsigned i = 0; i < Dimensions; i++)
            {
                assert(lower_limits[i] < upper_limits[i]);
            }
        }

        [[nodiscard]] bool Contains(const VectorN &point) const override
        {
            // If all upper and lower boundaries are satisfied we are inside
            return ((point.array() > (this->center_ + this->lower_limits_).array()).all() &&
                    (point.array() < (this->center_ + this->upper_limits_).array()).all());
        }

        [[nodiscard]] bool IsAtBoundary(const VectorN &point) const override
        {
            // Any dimension within tolerance of a boundary means our point is at a boundary
            return (((point - (this->center_ + this->lower_limits_)).cwiseAbs().array() <= this->tol_).any() ||
                    (((this->center_ + this->upper_limits_) - point).cwiseAbs().array() <= this->tol_).any());
        }

        [[nodiscard]] VectorN GetNearestPointWithinBound(const VectorN &point, const VectorN &prev_point) const override
        {
            // In 1D the solution will be one of the limits
            if constexpr (Dimensions == 1)
            {
                // Below lower limit
                if (point[0] < (this->center_[0] + this->lower_limits_[0]))
                {
                    // Snap to lower limits
                    return this->lower_limits_;
                }
                // Above upper limit
                else if (point[0] > (this->center_[0] + this->upper_limits_[0])) // Above upper limit
                {
                    // Snap to lower limits
                    return this->upper_limits_;
                }
                // Else We are already in the bounds
                else
                {
                    return point;
                }
            }

            // There might be a smarter math way to do higher dimensions considering intersections of hyperplanes and hypershapes, but for now we can do this numerically
            else
            {
                return BoundBase<Dimensions, Scalar>::GetNearestPointWithinBound(point, prev_point);
            }
        }

        [[nodiscard]] std::vector<VectorN> GetSurfaceNormals(const VectorN &point) const override
        {
            // Build Boolean array for where we are at boundaries
            const VectorN combined_surface_vectors = ((point - (this->center_ + this->lower_limits_)).cwiseAbs().array() <= this->tol_).template cast<Scalar>() -
                                                     (((this->center_ + this->upper_limits_) - point).cwiseAbs().array() <= this->tol_).template cast<Scalar>();
            std::vector<VectorN> surface_normals;

            for (unsigned i = 0; i < Dimensions; i++)
            {
                if (abs(combined_surface_vectors[i]) > this->tol_)
                {
                    surface_normals.push_back(-combined_surface_vectors[i] * VectorN::Unit(i));
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