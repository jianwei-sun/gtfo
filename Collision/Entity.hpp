//----------------------------------------------------------------------------------------------------
// File: Entity.hpp
// Desc: class representing a physical entity upon which collisions can occur
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <vector>
#include <utility>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "Segment.hpp"

#include <iostream>
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
class Entity{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Entity(const std::vector<Vector3>& vertices, const bool& fixed)
        :   vertices_(vertices),
            fixed_(fixed)
    {
        // Ensure at least one vertex exists
        assert(vertices_.size() >= 1);
        UpdateSegments();
    }

    bool IsFixed(void) const{
        return fixed_;
    }

    virtual void UpdateVertices(const std::vector<Vector3>& vertices) = 0;

    void UpdateSegments(void){
        segments_.clear();

        if(vertices_.size() == 1){
            segments_.push_back(Segment(vertices_[0]));
        }
        
        if(vertices_.size() >= 2){
            segments_.reserve(vertices_.size() - 1);
            for(size_t i = 0; i < vertices_.size() - 1; ++i){
                segments_.push_back(Segment(vertices_[i], vertices_[i + 1]));
            }
        }
    }

    void ClearCollisions(void){
        collisions_.clear();
    }

    void ComputeCollisions(const Entity& other, const Scalar& tol){
        for(size_t i = 0; i < segments_.size(); ++i){
            for(const Segment<Scalar>& segment_other : other.segments_){
                const Segment<Scalar> potential_collision_vector = segments_[i].MinDistanceVectorTo(segment_other);
                // std::cout << potential_collision_vector.Length() << "----" << i << std::endl;
                // std::cout << " _______________________________" << std::endl;
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
    std::vector<Segment<Scalar>> segments_;
    std::vector<Collision<Scalar>> collisions_;

private:
    const bool fixed_;
};

}   // namespace collision
}   // namespace gtfo