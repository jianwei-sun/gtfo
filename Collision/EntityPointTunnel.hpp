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
struct SegmentParams{
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    Scalar proj = 0;
    Vector3 dist = Vector3::Zero();
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

template<unsigned int JointSpaceDimension, typename Scalar = double, unsigned int VirtualDimension = JointSpaceDimension>
class EntityPointTunnel{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using VirtualVector = Eigen::Matrix<Scalar, VirtualDimension, 1>;
    // for arm
    EntityPointTunnel(const std::vector<Vector3>& vertices, const bool& fixed)
        :   vertices_elbow_(std::vector<Vector3> {vertices[0]}),
            vertices_wrist_(std::vector<Vector3> {vertices[1]}),
            fixed_(fixed),
            dir_nor_(),
            index_(0),
            closest_point_(Vector3::Zero())
    {
        // Ensure at least one vertex exists
        assert(vertices.size() >= 1);
    }

    // for tunnel
    EntityPointTunnel(const bool& fixed)
        :   vertices_elbow_(),
            vertices_wrist_(),
            fixed_(fixed),
            dir_nor_(),
            index_(0),
            closest_point_(Vector3::Zero())
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

    void ComputeCollisions(const EntityPointTunnel& other, const Scalar& radius, bool enable_tunnel){
        MinDistanceVectorTo(potential_collision_vector_elbow_, vertices_elbow_[0], other.vertices_elbow_, other.dir_nor_, radius);
        
        // if(potential_collision_vector_elbow_.has_normal_contact_tan && enable_tunnel){
        //     collisions_elbow_.emplace_back(vertices_elbow_[0], - potential_collision_vector_elbow_.normal_contact_direction_tan);
        // }

        if(potential_collision_vector_elbow_.has_normal_contact_nor){
            collisions_elbow_.emplace_back(vertices_elbow_[0], - potential_collision_vector_elbow_.normal_contact_direction_nor);
        }

        // if(potential_collision_vector_elbow_.has_tangential_contact){
        //     collisions_elbow_.emplace_back(vertices_elbow_[0], - potential_collision_vector_elbow_.tangential_contact_direction);
        // }

        MinDistanceVectorTo(potential_collision_vector_wrist_, vertices_wrist_[0], other.vertices_wrist_, other.dir_nor_, radius, 1);
        if(potential_collision_vector_wrist_.has_normal_contact_tan && enable_tunnel){
            collisions_wrist_.emplace_back(vertices_wrist_[0], - potential_collision_vector_wrist_.normal_contact_direction_tan);
        }

        if(potential_collision_vector_wrist_.has_normal_contact_nor){
            collisions_wrist_.emplace_back(vertices_wrist_[0], - potential_collision_vector_wrist_.normal_contact_direction_nor);
        }

        // if(potential_collision_vector_wrist_.has_tangential_contact){
        //     collisions_wrist_.emplace_back(vertices_wrist_[0], - potential_collision_vector_wrist_.tangential_contact_direction);
        // }
    }

    SegmentParams<Scalar> ParseSegment(const Vector3& start_point, const Vector3& end_point, const Vector3& point_of_interest){
        SegmentParams results;
        Vector3 dir = end_point - start_point;
        Vector3 start_to_point = point_of_interest - start_point;
        results.proj = start_to_point.dot(dir)/dir.squaredNorm();
        if (results.proj < 0){
            results.dist = start_point - point_of_interest;
        } else if (results.proj > 1){
            results.dist = end_point - point_of_interest;
        } else {
            results.dist = start_point + results.proj * dir - point_of_interest;
        }
        return results;
    }

    void MinDistanceVectorTo(CollisionVector<Scalar>& potential_collision_vector, const Vector3& point_of_interest, const std::vector<Vector3>& other, const Vector3& dir_nor, const Scalar& radius, const bool get_closest = 0) {
        Scalar min_dist_sq = std::numeric_limits<Scalar>::max();
        Vector3 min_dist_vector;
        Scalar min_proj;
        SegmentParams results;
        int index;
        for (int i = 0; i < other.size() - 1; ++i) {
            results = ParseSegment(other[i], other[i+1], point_of_interest);
            Scalar dist_sq = results.dist.squaredNorm();
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                min_dist_vector = results.dist;
                min_proj = results.proj;
                index = i;
            }
        }

        if (index != -1) {
            index_ = index;
            results_ = results;
            if(get_closest){
                if (results.proj < 0){
                    closest_point_ = other[index_];
                } else if (results.proj > 1){
                    closest_point_ = other[index_ + 1];
                } else{
                    closest_point_ = other[index_] + results.proj * (other[index_ + 1] - other[index_]);
                }
            }
            potential_collision_vector.has_tangential_contact = 0;
            potential_collision_vector.has_normal_contact_tan = 0;
            potential_collision_vector.has_normal_contact_nor = 0;
            potential_collision_vector.tangential_contact_direction = Vector3::Zero();
            potential_collision_vector.normal_contact_direction_tan = Vector3::Zero();
            potential_collision_vector.normal_contact_direction_nor = Vector3::Zero();
            potential_collision_vector.hit_end_wall = 0;
            // when there is contact on the ends
            Scalar normal_distance;
            Vector3 tan = Vector3::Zero();

            potential_collision_vector.normal_contact_direction_nor = (min_dist_vector.dot(dir_nor) * dir_nor).normalized();
            normal_distance = (min_dist_vector.dot(dir_nor) * dir_nor).norm();
            if (normal_distance > 0) {
                potential_collision_vector.has_normal_contact_nor = 1;
            } 

            // distance on the plane of interest
            Vector3 displacement = min_dist_vector - min_dist_vector.dot(dir_nor)* dir_nor;
            // check the contacts with the end caps
             
            if ((index_ == other.size() - 2 && min_proj > 1) || (index_ == 0 && min_proj < 0)){
                if (index_ == other.size() - 2) { 
                    tan = (other[index_] - other[index_+1]).normalized();
                } else { 
                    tan = (other[index_+1] - other[index_]).normalized();
                } 

                Scalar tangential_displacement = displacement.dot(tan);
                // Vector3 normal_displacement = displacement - tangential_displacement * tan;
                // Vector3 normal_displacement = displacement;

                if (tangential_displacement >= 0) { // 
                    potential_collision_vector.has_tangential_contact = 1;
                    potential_collision_vector.tangential_contact_direction = tan;

                    if (index_ == other.size() - 2) {
                        potential_collision_vector.hit_end_wall = 1; 
                    }
                } 

                // if(radius - normal_displacement.norm() <= 0) {
                //     potential_collision_vector.has_normal_contact_tan = 1;
                //     potential_collision_vector.normal_contact_direction_tan = normal_displacement.normalized();
                // }

            } 

            if(radius - displacement.norm() <= 0) {
                potential_collision_vector.has_normal_contact_tan = 1;
                potential_collision_vector.normal_contact_direction_tan = displacement.normalized();
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

    Vector3 GetTangentialDirection(const std::vector<Vector3>& other) const{
        return (other[index_ + 1] - other[index_]).normalized();
    }

    Vector3 GetClosestWristPoint(void) const{
        return closest_point_; 
    }

    double GetPositionError(const Vector3& point_of_interest, const std::vector<Vector3>& other) {
        Scalar error = 0;
        int index_desired = 0;
        int index_current = index_;
        Scalar min_dist_sq = std::numeric_limits<Scalar>::max();
        Vector3 min_dist_vector;
        Scalar min_proj;
        SegmentParams results; // for the desired
        for (int i = 0; i < other.size() - 1; ++i) {
            results = ParseSegment(other[i], other[i+1], point_of_interest);
            Scalar dist_sq = results.dist.squaredNorm();
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                min_dist_vector = results.dist;
                min_proj = results.proj;
                index_desired = i;
            }
        }
        
        // uniform results
        if (results_.proj > 1){
            index_current = index_current + 1;
            results_.proj -= 1;
        }

        if (results.proj > 1){
            index_desired = index_desired + 1;
            results.proj -= 1;
        }

        Vector3 current_point_on_trajectory;
        Vector3 desired_point_on_trajectory;
        if (results_.proj >= 0 && results_.proj <= 1){
            current_point_on_trajectory = other[index_current] + results_.proj * (other[index_current + 1] - other[index_current]);
        } else {
            current_point_on_trajectory = other[index_current];
        }

        if (results.proj >= 0 && results.proj <= 1){
            desired_point_on_trajectory = other[index_desired] + results.proj * (other[index_desired + 1] - other[index_desired]);
        } else {
            desired_point_on_trajectory = other[index_desired];
        }

        // if on the same segment
        if (index_desired == index_current){
            error = (desired_point_on_trajectory - current_point_on_trajectory).norm();
        } else if (index_desired > index_current) {
            for (int i = index_current; i <= index_desired; ++i){
                if (i == index_current){
                    error += (other[i+1] - current_point_on_trajectory).norm();
                } else if (i == index_desired){
                    error += (desired_point_on_trajectory - other[i]).norm();
                } else {
                    error += (other[i+1] - other[i]).norm();
                }
            }
        } else {
            for (int i = index_desired; i <= index_current; ++i){
                if (i == index_desired){
                    error += (other[i+1] - current_point_on_trajectory).norm();
                } else if (i == index_current){
                    error += (desired_point_on_trajectory - other[i]).norm();
                } else {
                    error += (other[i+1] - other[i]).norm();
                }
            }
        }
        
        int motion_direction = 1;
        if (index_desired - index_current > 0){
            motion_direction = 1;
        } else if (index_desired - index_current < 0) {
            motion_direction = -1;
        } else {
            if (results.proj - results_.proj > 0) {
                motion_direction = 1;
            } else if (results.proj - results_.proj < 0){
                motion_direction = -1;
            } else {
                motion_direction = 0;
            }
        }
        return motion_direction * error;
    }

    virtual void UpdateVirtualState(void) = 0;
    virtual void UpdateVirtualState(const VirtualVector& new_position) = 0;

protected:
    std::vector<Vector3> vertices_elbow_;
    std::vector<Vector3> vertices_wrist_;
    std::vector<Collision<Scalar>> collisions_elbow_;
    std::vector<Collision<Scalar>> collisions_wrist_;
    CollisionVector<Scalar> potential_collision_vector_elbow_;
    CollisionVector<Scalar> potential_collision_vector_wrist_;
    Vector3 dir_nor_;
    int index_;
    SegmentParams<Scalar> results_;
    Vector3 closest_point_;
private:
    const bool fixed_;
};

}   // namespace collision
}   // namespace gtfo