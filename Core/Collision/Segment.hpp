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
        if(this->IsPoint() && !other.IsPoint()){
            const Vector3& point = start_;
            const Scalar ratio = other.RatioOfPoint(point, true);
            return Segment(start_, other.PointAtRatio(ratio));
        }

        // Line to point
        if(!this->IsPoint() && other.IsPoint()){
            const Vector3& point = other.start_;
            const Scalar ratio = this->RatioOfPoint(point, true);
            return Segment(this->PointAtRatio(ratio), point);
        }

        // Line to line
        //     First check whether the lines are parallel
        //     If so, get the midpoint of the overlapped region
        //     Otherwise, get the mutual perpendicular
        //     https://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments
        const Vector3 self_vector = end_ - start_;
        const Vector3 other_vector = other.end_ - other.start_;
        const Vector3 between_vector = other.start_ - start_;
        const Vector3 normalized_cross = self_vector.normalized().cross(other_vector.normalized());

        // First check if the segments are parallel
        if(IsEqual(normalized_cross, Vector3::Zero())){

            // Find the midpoint of the overlapping sections
            const Scalar other_start_ratio = this->RatioOfPoint(other.start_);
            const Scalar other_end_ratio = this->RatioOfPoint(other.end_);

            // Get the start and end ratios of the overlapping region, regardless of the orientation of other
            const Scalar overlap_start_ratio = std::max(std::min(other_start_ratio, other_end_ratio), 0.0);
            const Scalar overlap_end_ratio = std::min(std::max(other_start_ratio, other_end_ratio), 1.0);

            // Get the midpoint of the overlapped region
            const Vector3 point_at_overlap_midpoint = this->PointAtRatio((overlap_start_ratio + overlap_end_ratio) / 2.0);
            return Segment(point_at_overlap_midpoint, other.PointAtRatio(other.RatioOfPoint(point_at_overlap_midpoint)));
        } 
        // If not parallel, then compute the mutual perpendicular segment
        else{
            const Scalar self_ratio = (Eigen::Matrix<Scalar, 3, 3>() << between_vector, other_vector.normalized(), normalized_cross).finished().determinant() / (normalized_cross.dot(normalized_cross) * self_vector.norm());
            const Scalar other_ratio = (Eigen::Matrix<Scalar, 3, 3>() << between_vector, self_vector.normalized(), normalized_cross).finished().determinant() / (normalized_cross.dot(normalized_cross) * other_vector.norm());

            // Then clamp to endpoints
            return Segment(this->PointAtRatio(self_ratio, true), other.PointAtRatio(other_ratio, true));
        }
    }

private:
    Scalar RatioOfPoint(const Vector3& point, const bool& clamp = false) const{
        const Vector3 segment_vector = end_ - start_;
        const Scalar ratio = segment_vector.dot(point - start_) / segment_vector.dot(segment_vector);
        if(clamp){
            return Eigen::Matrix<Scalar, 1, 1>(ratio).cwiseMax(0.0).cwiseMin(1.0).value();
        } else{
            return ratio;
        }
    }

    Vector3 PointAtRatio(const Scalar& ratio, const bool& clamp = false) const{
        if(clamp){
            return start_ + Eigen::Matrix<Scalar, 1, 1>(ratio).cwiseMax(0.0).cwiseMin(1.0).value() * (end_ - start_);
        } else{
            return start_ + ratio * (end_ - start_);
        }
    }

    Vector3 start_, end_;
};

}   // namespace collision
}   // namespace gtfo