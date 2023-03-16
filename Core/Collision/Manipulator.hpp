//----------------------------------------------------------------------------------------------------
// File: Manipulator.hpp
// Desc: class representing a physical manipulator
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <vector>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "Entity.hpp"

namespace gtfo{
namespace collision{

template<typename Scalar = double>
class Manipulator : public Entity<Scalar>{
public:
    using Vector3 = typename Entity<Scalar>::Vector3;

    Manipulator(const std::vector<Vector3>& vertices)
        :   Entity<Scalar>(vertices, false),
            number_of_vertices_(vertices.size())
    {
        
    }

    void UpdateVertices(const std::vector<Vector3>& vertices) override{
        assert(vertices.size() == number_of_vertices_);
        Entity<Scalar>::vertices_ = vertices;
        Entity<Scalar>::UpdateSegments();
    }

private:
    const size_t number_of_vertices_;
};

}   // namespace collision
}   // namespace gtfo