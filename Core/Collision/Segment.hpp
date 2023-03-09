//----------------------------------------------------------------------------------------------------
// File: Segment.hpp
// Desc: class representing a physical link segment
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Utils/Comparisons.hpp"

namespace gtfo{
namespace collision{

template<typename Scalar = double>
class Segment{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Segment(const Vector3& start, const Vector3& end = Vector3::Constant(NAN))
        :   start_(start),
            end_(end)
    {

    }

    Vector3 Start(void) const{
        return start_;
    }

    Vector3 End(void) const{
        return end_;
    }

    bool IsPoint(void) const{
        return end_.array().isNaN().any();
    }

    Scalar Length(void) const{
        return (end_ - start_).norm();
    }

    Segment MinDistanceVectorTo(const Segment& other) const{
        // Point to point
        if(this->IsPoint() && other.IsPoint()){
            return Segment(start_, other.start_);
        }

        // Point to line
        //    Found by projecting the point onto the line containing the segment
        //    and checking if it falls within the segment
        //    https://stackoverflow.com/a/1501725
        // TODO: fix length squared
        if(this->IsPoint() && !other.IsPoint()){
            const Vector3& point = start_;
            const Scalar ratio = Eigen::Matrix<Scalar, 1, 1>((point - other.start_).dot(other.end_ - other.start_) / other.Length()).cwiseMax(0.0).cwiseMin(1.0).value();
            return Segment(start_, other.start_ + ratio * (other.end_ - other.start_));
        }

        // Line to point
        if(!this->IsPoint() && other.IsPoint()){
            const Vector3& point = other.start_;
            const Scalar ratio = Eigen::Matrix<Scalar, 1, 1>((point - start_).dot(end_ - start_) / this->Length()).cwiseMax(0.0).cwiseMin(1.0).value();
            return Segment(start_ + ratio * (end_ - start_), point);
        }

        // Line to line
        //     https://stackoverflow.com/a/67102941/7338620
        const Vector3 segment_self = end_ - start_;
        const Vector3 segment_other = other.end_ - other.start_;
        const Vector3 separation = other.start_ - start_;

        // Precompute dot products
        const Scalar self_along_separation = segment_self.dot(separation);
        const Scalar other_along_separation = segment_other.dot(separation);
        const Scalar self_along_other = segment_self.dot(segment_other);
        const Scalar self_along_self = segment_self.dot(segment_self);
        const Scalar other_along_other = segment_other.dot(segment_other);
        
        const Scalar determinant = self_along_self * other_along_other - self_along_other * self_along_other;

        Scalar ratio_along_self = 0.0;
        Scalar ratio_along_other = 0.0;

        if(determinant < GTFO_EQUALITY_COMPARISON_TOLERANCE * self_along_self * other_along_other){
            // ratio_along_self = Eigen::Matrix<Scalar, 1, 1>(self_along_separation / self_along_self).cwiseMax(0.0).cwiseMin(1.0).value();
            // TODO: When the segments are parallel, return the answer as the midpoints between the overlapped regions
            
            const Scalar a = (other.start_ - start_).dot(segment_self) / self_along_self;
            const Scalar b = (other.end_ - start_).dot(segment_self) / self_along_self;

            ratio_along_self = (std::max(std::min(a, b), 0.0) + 
            std::min(std::max(a, b), 1.0)) / 2.0;

            const Scalar c = (start_ - other.start_).dot(segment_other) / other_along_other;
            const Scalar d = (end_ - other.start_).dot(segment_other) / other_along_other;

            ratio_along_other = (std::max(std::min(c, d), 0.0) + 
            std::min(std::max(c, d), 1.0)) / 2.0;


            return Segment(start_ + ratio_along_self * segment_self, other.start_ + ratio_along_other * segment_other);
        } else{
            ratio_along_self = Eigen::Matrix<Scalar, 1, 1>((self_along_separation * other_along_other - other_along_separation * self_along_other) / determinant).cwiseMax(0.0).cwiseMin(1.0).value();
            ratio_along_other = Eigen::Matrix<Scalar, 1, 1>((self_along_separation * self_along_other - other_along_separation * self_along_self) / determinant).cwiseMax(0.0).cwiseMin(1.0).value();
        }

        const Scalar scaled_distance_along_self = Eigen::Matrix<Scalar, 1, 1>((ratio_along_self * self_along_other + self_along_separation) / self_along_self).cwiseMax(0.0).cwiseMin(1.0).value();
        const Scalar scaled_distance_along_other = Eigen::Matrix<Scalar, 1, 1>((ratio_along_other * self_along_other - other_along_separation) / other_along_other).cwiseMax(0.0).cwiseMin(1.0).value();

        return Segment(start_ + scaled_distance_along_self * segment_self, other.start_ + scaled_distance_along_other * segment_other);
    }

private:
    Vector3 start_, end_;
};

}   // namespace collision
}   // namespace gtfo