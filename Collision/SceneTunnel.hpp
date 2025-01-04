//----------------------------------------------------------------------------------------------------
// File: SceneTunnel.hpp
// Desc: class representing a physical scene involving collidable entities
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <vector>
#include <memory>
#include <type_traits>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "EntityPointTunnel.hpp"

namespace gtfo{
namespace collision{

template<typename Scalar = double>
class SceneTunnel{
public:
    using Vector3 = typename EntityPointTunnel<Scalar>::Vector3;
    using EntityPtr = std::shared_ptr<EntityPointTunnel<Scalar>>;

    template<typename... T>
    SceneTunnel(const T&... entity)
    {
        static_assert(std::conjunction_v<std::is_base_of<EntityPointTunnel<Scalar>, T>...>, "SceneTunnel arguments must inherit from Entity");
        ([&]{
            if(entity.IsFixed()){
                fixed_entities_.push_back(std::make_shared<T>(entity));
            } else{
                free_entities_.push_back(std::make_shared<T>(entity));
            }
        }(), ...);
    }


    template<typename T>
    void AddEntity(const T& entity){
        static_assert(std::is_base_of_v<EntityPointTunnel<Scalar>, T>, "Entities must inherit from Entity");
        if(entity.IsFixed()){
            if (fixed_entities_.size() > 0) { //specific to project. only want one fixed entity which is the pair of tunnels
                fixed_entities_.clear(); // clear fixed_entities_
            }
            fixed_entities_.push_back(std::make_shared<T>(entity)); 
        } else{
            free_entities_.push_back(std::make_shared<T>(entity));
        }
    }

    // Need mechanism for adding exclusions to segment pair collision checking

    void ComputeCollisions(const Scalar& radius){
        // For each free entity
        for (int i = 0; i < free_entities_.size(); i ++){
            free_entities_[i]->ClearCollisions();
            free_entities_[i]->ComputeCollisions(*fixed_entities_[i], radius);
        }
    }

    EntityPtr GetFreeEntity(const size_t& free_entities_index) const{
        return free_entities_.at(free_entities_index);
    }

    std::vector<Collision<Scalar>> GetElbowCollisions(const size_t& free_entities_index) const{
        return free_entities_.at(free_entities_index)->GetElbowCollisions();
    }

    std::vector<Collision<Scalar>> GetWristCollisions(const size_t& free_entities_index) const{
        return free_entities_.at(free_entities_index)->GetWristCollisions();
    }

    CollisionVector<Scalar> GetElbowCollisionVector(const size_t& free_entities_index) const{
        return free_entities_.at(free_entities_index)->GetElbowCollisionVector();
    }

    CollisionVector<Scalar> GetWristCollisionVector(const size_t& free_entities_index) const{
        return free_entities_.at(free_entities_index)->GetWristCollisionVector();
    }

    Vector3 GetTangentialDirection(const size_t& free_entities_index, const std::vector<Vector3>& other) const{
        return free_entities_.at(free_entities_index)->GetTangentialDirection(other);
    }

    double GetPositionError(const size_t& free_entities_index, const Vector3& point_of_interest, const std::vector<Vector3>& other) {
        return free_entities_.at(free_entities_index)->GetPositionError(point_of_interest, other);
    }

    void UpdateVertices(const size_t& free_entities_index, const std::vector<Vector3>& vertices){
        free_entities_.at(free_entities_index)->UpdateVertices(vertices);
    }

    // Assumed that this function is called after ComputeCollisions has executed
    void UpdateVirtualState(void){
        for(EntityPtr& free_entity : free_entities_){
            free_entity->UpdateVirtualState();
        }
    }

    // Getter for free entity list
    std::vector<EntityPtr> GetFreeEntityList() const
    {
        return free_entities_;
    }

    // Getter for fixed entity list
    std::vector<EntityPtr> GetFixedEntityList() const
    {
        return fixed_entities_;
    }

private:
    std::vector<EntityPtr> fixed_entities_;
    std::vector<EntityPtr> free_entities_;
};

}   // namespace collision
}   // namespace gtfo