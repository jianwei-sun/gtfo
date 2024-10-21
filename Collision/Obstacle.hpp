//----------------------------------------------------------------------------------------------------
// File: Obstacle.hpp
// Desc: class representing a physical obstacle
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <vector>

// Third-party dependencies
#include <Eigen/Dense>

// // Project-specific
// #include "Entity.hpp"

// namespace gtfo{
// namespace collision{

// template<typename Scalar = double>
// class Obstacle : public Entity<Scalar>{
// public:
//     using Vector3 = typename Entity<Scalar>::Vector3;

//     Obstacle(const std::vector<Vector3>& vertices)
//         :   Entity<Scalar>(vertices, true)
//     {

//     }

//     void UpdateVertices(const std::vector<Vector3>& vertices) override{
//         // Empty, since an obstacle is fixed.
//     }

//     void UpdateVirtualState(void) override{
//         // Empty, since there are no dynamics
//     }
// };

// }   // namespace collision
// }   // namespace gtfo