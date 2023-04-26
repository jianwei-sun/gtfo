//----------------------------------------------------------------------------------------------------
// File: Entity.hpp
// Desc: class representing a physical entity upon which collisions can occur
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <vector>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "Segment.hpp"

namespace gtfo{
namespace collision{
    
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
        for(const Segment<Scalar>& segment_self : segments_){
            for(const Segment<Scalar>& segment_other : other.segments_){
                const Segment<Scalar> potential_collision_vector = segment_self.MinDistanceVectorTo(segment_other);
                if(potential_collision_vector.Length() <= tol){
                    collisions_.push_back(potential_collision_vector);
                }
            }
        }
    }

    std::vector<Segment<Scalar>> GetCollisions(void) const{
        return collisions_;
    }

protected:
    std::vector<Vector3> vertices_;
    std::vector<Segment<Scalar>> segments_;
    std::vector<Segment<Scalar>> collisions_;

private:
    const bool fixed_;
};

}   // namespace collision
}   // namespace gtfo