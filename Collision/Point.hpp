//----------------------------------------------------------------------------------------------------
// File: Point.hpp
// Desc: class representing a physical elbow and wrist positions
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes

// Third-party dependencies
#include <Eigen/Dense>
#include "VirtualTunnel.hpp"

namespace gtfo{
namespace collision{

template<typename Scalar = double>
class Point{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Point(const Vector3& point_of_interest, const Vector3& point_on_curve)
        :   point_of_interest_(point_of_interest),
            point_on_curve_(point_on_curve)
    {

    }

    Vector3 PointOfInterest(void) const{
        return point_of_interest_;
    }

    Vector3 PointOnCurve(void) const{
        return point_on_curve_;
    }

    Scalar Length(void) const{
        return (point_on_curve_ - point_of_interest_).norm();
    }

    Point MinDistanceVectorTo(const Eigen::MatrixXd other) const{
        double min_dist = std::numeric_limits<double>::max();
        int index = -1;
        for (int i = 0; i < other.rows(); ++i) {
            double dist = (other.row(i) - point_of_interest_.transpose()).norm();
            if (dist < min_dist) {
                min_dist = dist;
                index = i;
            }
        }
        point_on_curve_ =  other.row(index).transpose();
        return Point(point_of_interest_, point_on_curve_);
    }

private:
    Vector3 point_of_interest_;
    Vector3 point_on_curve_;
};

}   // namespace collision
}   // namespace gtfo