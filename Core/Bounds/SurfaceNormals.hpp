//----------------------------------------------------------------------------------------------------
// File: SurfaceNormals.hpp
// Desc: base class for boolean expressions of surface normals
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <numeric>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Utils/Comparisons.hpp"
#include "../Utils/Containers.hpp"

namespace gtfo {

template <typename VectorN>
class SurfaceNormals : public std::vector<VectorN>{
public:
    using Scalar = typename Eigen::DenseBase<VectorN>::Scalar;

    SurfaceNormals() = default;
    SurfaceNormals(const VectorN& vector) : std::vector<VectorN>{vector} {}
    SurfaceNormals(const std::vector<VectorN> vectors) : std::vector<VectorN>(vectors) {}

    [[nodiscard]] bool Contains(const VectorN& vector) const{
        return ContainsVector(*this, vector);
    }

    [[nodiscard]] bool HasPositiveDotProductWith(const VectorN& vector) const{
        for(const VectorN& surface_normal : *this){
            if(vector.dot(surface_normal) > 0.0){
                return true;
            }
        } 
        return false;
    }

    void RemoveComponentIn(VectorN& vector) const {
        for(const VectorN& surface_normal : *this){
            const Scalar dot_product = vector.dot(surface_normal);
            if(dot_product > 0.0){
                vector -= dot_product * surface_normal;
            }
        }
    }
};

}   // namespace gtfo