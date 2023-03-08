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

    bool IsPoint(void) const{
        return end_.array().isNan().any();
    }

    Scalar Length(void) const{
        return (end_ - start_).norm();
    }

    Segment MinDistanceVectorTo(const Segment& other){
        // Point to point
        if(this->IsPoint() && other.IsPoint()){
            return Segment(start_, other.start_);
        }

        // Point to line
        if(this->IsPoint() && !other.IsPoint()){

        }

        // Line to point
        if(!this->IsPoint() && other.IsPoint()){

        }

        // Line to line
        // Otherwise follow this concise answer on stackoverflow:
        // https://stackoverflow.com/a/67102941/7338620
    }

private:
    Vector3 start_, end_;
};

}   // namespace collision
}   // namespace gtfo