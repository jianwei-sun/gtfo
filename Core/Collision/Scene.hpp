//----------------------------------------------------------------------------------------------------
// File: Scene.hpp
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
#include "Entity.hpp"
#include "Segment.hpp"

namespace gtfo{
namespace collision{

template<typename Scalar = double>
class Scene{
public:
    using Vector3 = typename Entity<Scalar>::Vector3;
    using EntityPtr = std::shared_ptr<Entity<Scalar>>;

    template<typename... T>
    Scene(const T&... entity)
        :   free_to_fixed_tol_(0.5),
            free_to_free_tol_(0.5)
    {
        static_assert(std::conjunction_v<std::is_base_of<Entity<Scalar>, T>...>, "Scene arguments must inherit from Entity");
        ([&]{
            if(entity.IsFixed()){
                fixed_entities_.push_back(std::make_shared<Entity<Scalar>>(entity));
            } else{
                free_entities_.push_back(std::make_shared<Entity<Scalar>>(entity));
            }
        }(), ...);
    }

    void SetCollisionDetectionThresholds(const Scalar& free_to_fixed_tol, const Scalar& free_to_free_tol){
        assert(free_to_fixed_tol > 0.0);
        assert(free_to_free_tol > 0.0);

        free_to_fixed_tol_ = free_to_fixed_tol;
        free_to_free_tol_ = free_to_free_tol;
    }

    void AddEntity(const Entity<Scalar>& entity){
        if(entity.IsFixed()){
            fixed_entities_.push_back(std::make_shared<Entity<Scalar>>(entity));
        } else{
            free_entities_.push_back(std::make_shared<Entity<Scalar>>(entity));
        }
    }

    // Need mechanism for adding exclusions to segment pair collision checking

    void ComputeCollisions(void){
        // For each free entity
        for(EntityPtr& free_entity : free_entities_){
            free_entity->ClearCollisions();

            // Check collisions with all fixed entities
            for(const EntityPtr& fixed_entity : fixed_entities_){
                free_entity->ComputeCollisions(*fixed_entity, free_to_fixed_tol_);
            }

            // Check collisions with other free entities
            for(const EntityPtr& other_free_entity : free_entities_){
                if(free_entity == other_free_entity){
                    continue;
                }
                free_entity->ComputeCollisions(*other_free_entity, free_to_free_tol_);
            }
        }
    }

    std::vector<Segment<Scalar>> GetCollisions(const size_t& free_entities_index){
        return free_entities_.at(free_entities_index)->GetCollisions();
    }

private:
    std::vector<EntityPtr> fixed_entities_;
    std::vector<EntityPtr> free_entities_;
    Scalar free_to_fixed_tol_;
    Scalar free_to_free_tol_;
};

}   // namespace collision
}   // namespace gtfo