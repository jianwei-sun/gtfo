//----------------------------------------------------------------------------------------------------
// File: EntityPointTunnel.hpp
// Desc: class representing a physical Entity upon which collisions can occur
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <vector>
#include <utility>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "Point.hpp"
#include "VirtualTunnel.hpp"

namespace gtfo{
namespace collision{
    
template<typename Scalar = double>
struct Collision{
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    size_t segment_index_;
    Vector3 location_;
    Vector3 direction_;

    Collision()
        :   segment_index_(0),
            location_(Vector3::Zero()),
            direction_(Vector3::Zero())
    {}

    Collision(const size_t& segment_index, const Vector3& location, const Vector3& direction)
        :   segment_index_(segment_index),
            location_(location),
            direction_(direction)
    {}
};

template<typename Scalar = double>
class EntityPointTunnel{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    EntityPointTunnel(const std::vector<Vector3>& vertices, const bool& fixed)
        :   vertices_(vertices),
            fixed_(fixed)
    {
        // Ensure at least one vertex exists
        assert(vertices_.size() >= 1);
        UpdatePoints();
    }

    bool IsFixed(void) const{
        return fixed_;
    }

    virtual void UpdateVertices(const std::vector<Vector3>& vertices) = 0;

    void UpdatePoints(void){
        if (!fixed_){
            points_.clear();
            for(size_t i = 0; i < vertices_.size() - 1; ++i){
                points_.push_back(Point(vertices_[i]));
            }
            
        }
    }

    void ClearCollisions(void){
        collisions_.clear();
    }

    void ComputeCollisions(const EntityPointTunnel& other, const Scalar& tol){
        for(size_t i = 0; i < points_.size(); ++i){
            for(const Segment<Scalar>& segment_other : other.points_){
                const Segment<Scalar> potential_collision_vector = points_[i].MinDistanceVectorTo(segment_other);
                if(potential_collision_vector.Length() <= tol){
                    collisions_.emplace_back(i, potential_collision_vector.Start(), potential_collision_vector.End() - potential_collision_vector.Start());
                }
            }
        }
    }

    std::vector<Collision<Scalar>> GetCollisions(void) const{
        return collisions_;
    }

    virtual void UpdateVirtualState(void) = 0;

protected:
    std::vector<Vector3> vertices_;
    std::vector<Segment<Scalar>> points_;
    std::vector<Collision<Scalar>> collisions_;

private:
    const bool fixed_;
};

}   // namespace collision
}   // namespace gtfo