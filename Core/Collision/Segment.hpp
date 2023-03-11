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

        // First check if the segments are parallel
        const Vector3 self_vector = end_ - start_;
        const Vector3 other_vector = other.end_ - other.start_;
        const Vector3 between_vector = other.start_ - start_;

        if(IsEqual(self_vector.cross(other_vector), Vector3::Zero())){
            // Find the midpoint of the overlapping sections
            const Scalar other_start_ratio = this->RatioOfPoint(other.start_);
            const Scalar other_end_ratio = this->RatioOfPoint(other.end_);

            const Scalar other_relative_start_ratio = std::min(other_start_ratio, other_end_ratio);
            const Scalar other_relative_end_ratio = std::max(other_end_ratio, other_end_ratio);

            const Scalar overlap_start_ratio = std::max(other_relative_start_ratio, 0.0);
            const Scalar overlap_end_ratio = std::min(other_relative_end_ratio, 1.0);

            const Vector3 point_at_overlap_midpoint = this->PointAtRatio((overlap_start_ratio + overlap_end_ratio) / 2.0);

            return Segment(point_at_overlap_midpoint, other.PointAtRatio(other.RatioOfPoint(point_at_overlap_midpoint)));
        }

        // If not parallel, verify intersection
        const Vector3 normalized_cross = self_vector.normalized().cross(other_vector.normalized());
        const Scalar self_ratio = (Eigen::Matrix<Scalar, 3, 3>() << between_vector, other_vector.normalized(), normalized_cross).finished().determinant() / normalized_cross.dot(normalized_cross);
        const Scalar other_ratio = (Eigen::Matrix<Scalar, 3, 3>() << between_vector, self_vector.normalized(), normalized_cross).finished().determinant() / normalized_cross.dot(normalized_cross);

        const Scalar self_ratio_clamped = Eigen::Matrix<Scalar, 1, 1>(self_ratio).cwiseMax(0.0).cwiseMin(1.0).value() * self_vector.norm();
        const Scalar other_ratio_clamped = Eigen::Matrix<Scalar, 1, 1>(other_ratio).cwiseMax(0.0).cwiseMin(1.0).value() * other_vector.norm();

        return Segment(this->PointAtRatio(self_ratio_clamped), other.PointAtRatio(other_ratio_clamped));


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
    Scalar RatioOfPoint(const Vector3& point, const bool& clamp = false) const{
        const Vector3 segment_vector = end_ - start_;
        const Scalar ratio = segment_vector.dot(point) / segment_vector.dot(segment_vector);
        if(clamp){
            return Eigen::Matrix<Scalar, 1, 1>(ratio).cwiseMax(0.0).cwiseMin(1.0).value();
        } else{
            return ratio;
        }
    }

    Vector3 PointAtRatio(const Scalar& ratio) const{
        return start_ + ratio * (end_ - start_);
    }

    Vector3 start_, end_;
};

}   // namespace collision
}   // namespace gtfo