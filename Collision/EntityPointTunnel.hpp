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
#include <limits>
#include <omp.h>
// Project-specific

namespace gtfo{
namespace collision{
    
template<typename Scalar = double>
struct CollisionVector{
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    bool has_tangential_contact = 0;
    bool has_normal_contact = 0;
    Vector3 tangential_contact_direction = Vector3::Zero();
    Vector3 normal_contact_direction = Vector3::Zero();
    bool hit_end_wall = 0;
};

template<typename Scalar = double>
struct Collision{
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    Vector3 location_;
    Vector3 direction_;

    Collision()
        :   
            location_(Vector3::Zero()),
            direction_(Vector3::Zero())
    {}

    Collision(const Vector3& location, const Vector3& direction)
        :   
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
    }

    EntityPointTunnel(const bool& fixed)
        :   vertices_(),
            fixed_(fixed)
    {
        
    }

    bool IsFixed(void) const{
        return fixed_;
    }

    virtual void UpdateVertices(const std::vector<Vector3>& vertices) = 0;

    void ClearCollisions(void){
        collisions_.clear();
    }

    void ComputeCollisions(const EntityPointTunnel& other, const Scalar& radius){
        CollisionVector<Scalar> potential_collision_vector;
        MinDistanceVectorTo(potential_collision_vector, vertices_[0], other.vertices_, radius);
        if(potential_collision_vector.has_normal_contact){
            collisions_.emplace_back(vertices_[0], - potential_collision_vector.normal_contact_direction);
        }
        if(potential_collision_vector.has_tangential_contact){
            collisions_.emplace_back(vertices_[0], - potential_collision_vector.tangential_contact_direction);
        }
        // std::cout << "normal " << potential_collision_vector.has_normal_contact << std::endl;
        // std::cout << "tan " << potential_collision_vector.has_tangential_contact << std::endl;
        // std::cout << "size " << collisions_.size() << std::endl;
    }

    void MinDistanceVectorTo(CollisionVector<Scalar>& potential_collision_vector, const Vector3& point_of_interest, const std::vector<Vector3>& other, const Scalar& radius) const {

        double min_dist_sq = std::numeric_limits<double>::max();  
        int index = -1;

        #pragma omp parallel for
        for (int i = 0; i < other.size(); ++i) {
            double dist_sq = (other[i] - point_of_interest).squaredNorm();  

            #pragma omp critical
            {
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    index = i;
                }
            }
        }

        if (index != -1) {
            potential_collision_vector.has_tangential_contact = 0;
            potential_collision_vector.has_normal_contact = 0;
            potential_collision_vector.tangential_contact_direction = Vector3::Zero();
            potential_collision_vector.normal_contact_direction = Vector3::Zero();
            potential_collision_vector.hit_end_wall = 0;

            double normal_distance;
            Eigen::Vector3d tan = Eigen::Vector3d::Zero();
            // when there is no contact on the ends
            if (index == other.size() - 1) { 
                tan = (other[index-1] - other[index]).normalized();
            } else if (index == 0) { 
                tan = (other[index+1] - other[index]).normalized();
            } else {
                potential_collision_vector.normal_contact_direction = (other[index] - point_of_interest).normalized();
                normal_distance = (other[index] - point_of_interest).norm();
                
                if (radius - normal_distance <= 0) {
                    potential_collision_vector.has_normal_contact = 1;
                } else {
                    potential_collision_vector.has_normal_contact = 0;
                }
                return; 
            }
            // when there is contact on the ends
            Eigen::Vector3d displacement = other[index] - point_of_interest;
            double tangential_displacement = displacement.dot(tan);
            Eigen::Vector3d normal_displacement = displacement - tangential_displacement * tan;

            if (tangential_displacement >= 0) { // 
                potential_collision_vector.has_tangential_contact = 1;
                potential_collision_vector.tangential_contact_direction = tan;

                if (index == other.size() - 1) {
                    potential_collision_vector.hit_end_wall = 1; 
                }
            } else {
                potential_collision_vector.has_tangential_contact = 0;
                potential_collision_vector.tangential_contact_direction.setZero(); 
            }

            potential_collision_vector.normal_contact_direction = normal_displacement.normalized();
            normal_distance = normal_displacement.norm();

            if (radius - normal_distance <= 0) {
                potential_collision_vector.has_normal_contact = 1;
            } else {
                potential_collision_vector.has_normal_contact = 0;
            }

        }
    }

    std::vector<Collision<Scalar>> GetCollisions(void) const{
        return collisions_;
    }

    virtual void UpdateVirtualState(void) = 0;

protected:
    std::vector<Vector3> vertices_;
    std::vector<Collision<Scalar>> collisions_;

private:
    const bool fixed_;
};

}   // namespace collision
}   // namespace gtfo