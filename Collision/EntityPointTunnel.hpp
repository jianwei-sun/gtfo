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
    bool has_normal_contact_tan = 0;
    bool has_normal_contact_nor = 0;
    Vector3 tangential_contact_direction = Vector3::Zero();
    Vector3 normal_contact_direction_tan = Vector3::Zero();
    Vector3 normal_contact_direction_nor = Vector3::Zero();
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
    // for arm
    EntityPointTunnel(const std::vector<Vector3>& vertices, const bool& fixed)
        :   vertices_elbow_(std::vector<Vector3> {vertices[0]}),
            vertices_wrist_(std::vector<Vector3> {vertices[1]}),
            fixed_(fixed),
            dir_nor_()
    {
        // Ensure at least one vertex exists
        assert(vertices.size() >= 1);
    }

    // for tunnel
    EntityPointTunnel(const bool& fixed)
        :   vertices_elbow_(),
            vertices_wrist_(),
            fixed_(fixed),
            dir_nor_()
    {
        
    }

    bool IsFixed(void) const{
        return fixed_;
    }

    virtual void UpdateVertices(const std::vector<Vector3>& vertices) = 0;

    void ClearCollisions(void){
        collisions_elbow_.clear();
        collisions_wrist_.clear();
    }

    void ComputeCollisions(const EntityPointTunnel& other, const Scalar& radius){
        MinDistanceVectorTo(potential_collision_vector_elbow_, vertices_elbow_[0], other.vertices_elbow_, other.dir_nor_, radius);
        if(potential_collision_vector_elbow_.has_normal_contact_tan){
            collisions_elbow_.emplace_back(vertices_elbow_[0], - potential_collision_vector_elbow_.normal_contact_direction_tan);
        }

        if(potential_collision_vector_elbow_.has_normal_contact_nor){
            collisions_elbow_.emplace_back(vertices_elbow_[0], - potential_collision_vector_elbow_.normal_contact_direction_nor);
        }

        if(potential_collision_vector_elbow_.has_tangential_contact){
            collisions_elbow_.emplace_back(vertices_elbow_[0], - potential_collision_vector_elbow_.tangential_contact_direction);
        }

        MinDistanceVectorTo(potential_collision_vector_wrist_, vertices_wrist_[0], other.vertices_wrist_, other.dir_nor_, radius);
        if(potential_collision_vector_wrist_.has_normal_contact_tan){
            collisions_wrist_.emplace_back(vertices_wrist_[0], - potential_collision_vector_wrist_.normal_contact_direction_tan);
        }

        if(potential_collision_vector_wrist_.has_normal_contact_nor){
            collisions_wrist_.emplace_back(vertices_wrist_[0], - potential_collision_vector_wrist_.normal_contact_direction_nor);
        }

        if(potential_collision_vector_wrist_.has_tangential_contact){
            collisions_wrist_.emplace_back(vertices_wrist_[0], - potential_collision_vector_wrist_.tangential_contact_direction);
        }
    }

    void MinDistanceVectorTo(CollisionVector<Scalar>& potential_collision_vector, const Vector3& point_of_interest, const std::vector<Vector3>& other, const Vector3& dir_nor, const Scalar& radius) const {
        Scalar min_dist_sq = std::numeric_limits<Scalar>::max();
        int index = -1;
        #pragma omp parallel
        {
            Scalar local_min_dist_sq = std::numeric_limits<Scalar>::max();
            int local_index = -1;

            #pragma omp for
            for (int i = 0; i < other.size(); ++i) {
                Scalar dist_sq = (other[i] - point_of_interest).squaredNorm();

                if (dist_sq < local_min_dist_sq) {
                    local_min_dist_sq = dist_sq;
                    local_index = i;
                }
            }

            #pragma omp critical
            {
                if (local_min_dist_sq < min_dist_sq) {
                    min_dist_sq = local_min_dist_sq;
                    index = local_index;
                }
            }
        }

        if (index != -1) {
            potential_collision_vector.has_tangential_contact = 0;
            potential_collision_vector.has_normal_contact_tan = 0;
            potential_collision_vector.has_normal_contact_nor = 0;
            potential_collision_vector.tangential_contact_direction = Vector3::Zero();
            potential_collision_vector.normal_contact_direction_tan = Vector3::Zero();
            potential_collision_vector.normal_contact_direction_nor = Vector3::Zero();
            potential_collision_vector.hit_end_wall = 0;
            // when there is contact on the ends
            Scalar tangent_distance;
            Scalar normal_distance;
            Vector3 tan = Vector3::Zero();
            potential_collision_vector.normal_contact_direction_nor = ((other[index] - point_of_interest).dot(dir_nor) * dir_nor).normalized();
            potential_collision_vector.normal_contact_direction_tan = (other[index] - point_of_interest - (other[index] - point_of_interest).dot(dir_nor)* dir_nor).normalized();
            
            normal_distance = ((other[index] - point_of_interest).dot(dir_nor) * dir_nor).norm();
            tangent_distance = (other[index] - point_of_interest - (other[index] - point_of_interest).dot(dir_nor)* dir_nor).norm();
            
            if (normal_distance > 0) {
                potential_collision_vector.has_normal_contact_nor = 1;
            } else {
                potential_collision_vector.has_normal_contact_nor = 0;
                potential_collision_vector.normal_contact_direction_nor.setZero(); 
            }

            if (radius - tangent_distance <= 0) {
                potential_collision_vector.has_normal_contact_tan = 1;
            } else {
                potential_collision_vector.has_normal_contact_tan = 0;
                potential_collision_vector.normal_contact_direction_tan.setZero(); 
            }

            // when there is contact on the ends
            if ((index == other.size() - 1) || (index == 0)){
                if (index == other.size() - 1) { 
                    tan = (other[index-1] - other[index]).normalized();
                } else { 
                    tan = (other[index+1] - other[index]).normalized();
                } 
            
                Vector3 displacement = other[index] - point_of_interest;
                Scalar tangential_displacement = displacement.dot(tan);
                Vector3 normal_displacement = displacement - tangential_displacement * tan;

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
            }
                

        }
    }

    std::vector<Collision<Scalar>> GetElbowCollisions(void) const{
        return collisions_elbow_;
    }

    std::vector<Collision<Scalar>> GetWristCollisions(void) const{
        return collisions_wrist_;
    }

    CollisionVector<Scalar> GetElbowCollisionVector(void) const{
        return potential_collision_vector_elbow_;
    }

    CollisionVector<Scalar> GetWristCollisionVector(void) const{
        return potential_collision_vector_wrist_;
    }

    virtual void UpdateVirtualState(void) = 0;

protected:
    std::vector<Vector3> vertices_elbow_;
    std::vector<Vector3> vertices_wrist_;
    std::vector<Collision<Scalar>> collisions_elbow_;
    std::vector<Collision<Scalar>> collisions_wrist_;
    CollisionVector<Scalar> potential_collision_vector_elbow_;
    CollisionVector<Scalar> potential_collision_vector_wrist_;
    Vector3 dir_nor_;
private:
    const bool fixed_;
};

}   // namespace collision
}   // namespace gtfo