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

    Point(const Vector3& point_of_interest)
        :   point_of_interest_(point_of_interest)
    {

    }

    Vector3 PointOfInterest(void) const{
        return point_of_interest_;
    }


    Vector3 MinDistanceVectorTo(const VirtualTunnel& other) const{
        
    }

private:
    Vector3 point_of_interest_;
};

}   // namespace collision
}   // namespace gtfo