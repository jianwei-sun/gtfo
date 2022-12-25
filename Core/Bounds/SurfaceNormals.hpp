//----------------------------------------------------------------------------------------------------
// File: SurfaceNormals.hpp
// Desc: base class for boolean expressions of surface normals
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
// #include <algorithm>
// #include <iterator>
// #include <memory>
// #include <unordered_set>
// #include <array>
#include <numeric>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Utils/BooleanExpression.hpp"
#include "../Utils/Comparisons.hpp"

namespace gtfo {

template <typename VectorType>
class SurfaceNormals : public BooleanExpression<VectorType>{
public:
    using BooleanExpression = BooleanExpression<VectorType>;
    using Scalar = typename Eigen::DenseBase<VectorType>::Scalar;

    SurfaceNormals() : BooleanExpression(Relation::Intersection) {}

    template <typename... InputTypes>
    SurfaceNormals(const Relation& relation, const InputTypes&... inputs) 
        : BooleanExpression(relation, inputs...) {}

    SurfaceNormals(const Relation& relation, const std::vector<SurfaceNormals>& collection)
        : BooleanExpression(relation, collection) {}

    SurfaceNormals(const Relation& relation, const std::vector<VectorType>& vectors)
        : BooleanExpression(relation, vectors) {}

    [[nodiscard]] bool Contains(const VectorType& vector) const{
        const std::function<bool(const VectorType&)> vector_equal = 
            [&vector](const VectorType& surface_normal)->bool{
                return IsEqual(surface_normal, vector);
            };

        const std::function<bool(const Relation&, const std::vector<bool>&)> combine_results = 
            [](const Relation& relation, const std::vector<bool>& results)->bool{
                if(relation == Relation::Union){
                    // True if any is true
                    return std::find(results.cbegin(), results.cend(), true) != results.cend();
                } else{
                    // True if all is true
                    return std::find(results.cbegin(), results.cend(), false) == results.cend();
                }
            };

        return MapReduce(vector_equal, combine_results);
    }

    [[nodiscard]] bool IsEmpty() const{
        const std::function<VectorType(const VectorType&)> identity = 
            [](const VectorType& vector)->VectorType{
                return vector;
            };

        const std::function<VectorType(const Relation&, const std::vector<VectorType>&)> cancel_opposing_vectors = 
            [](const Relation& relation, const std::vector<VectorType>& vectors)->VectorType{
                if(relation == Relation::Union){
                    std::vector<VectorType> combined_vectors;
                    for(const VectorType& vector : vectors){
                        // Check if there already is a surface normal that points in the opposite direction
                        const typename std::vector<VectorType>::iterator it = std::find_if(combined_vectors.begin(), combined_vectors.end(), [&vector](const VectorType& combined_vector)->bool{
                            return IsEqual(-vector, combined_vector);
                        });

                        // If there is, remove it
                        if(it != combined_vectors.end()){
                            combined_vectors.erase(it);
                        // Otherwise, insert the surface normal to the collection
                        } else{
                            combined_vectors.push_back(vector);
                        }
                    }
                    // If the combined_vectors is empty, or if it only contains zero vectors, then return a zero vector to indicate empty
                    if(std::all_of(combined_vectors.cbegin(), combined_vectors.cend(), 
                        [](const VectorType& vector)->bool{
                            return vector.isZero();
                        })
                    ){
                        return VectorType::Zero();
                    }
                } else{
                    // Similarly, in the intersection case, if the input vectors are empty or only contains zero vectors, then return zero
                    if(std::all_of(vectors.cbegin(), vectors.cend(), 
                        [](const VectorType& vector)->bool{
                            return vector.isZero();
                        })
                    ){
                        return VectorType::Zero();
                    }
                }

                // At this point, it means there are vectors that do not cancel out, so do not return a zero vector
                return VectorType::Ones();
            };

        return MapReduce(identity, cancel_opposing_vectors).isZero();
    }

    [[nodiscard]] VectorType GetProjectionOf(const VectorType& vector) const{
        const std::function<VectorType(const VectorType&)> compute_single_projection = 
            [&vector](const VectorType& surface_normal)->VectorType{
                const Scalar inner_product = vector.dot(surface_normal);
                if(inner_product > 0.0){
                    return surface_normal * inner_product;
                } else{
                    return VectorType::Zero();
                }
            };

        const std::function<VectorType(const Relation&, const std::vector<VectorType>&)> combine_projections = 
            [](const Relation& relation, const std::vector<VectorType>& results)->VectorType{
                if(
                    relation == Relation::Union || 
                    std::all_of(results.cbegin(), results.cend(), [](const VectorType& result)->bool{
                        return result.isZero();
                    }))
                {
                    return std::reduce(results.cbegin(), results.cend());
                } else{
                    return VectorType::Zero();
                }
            };

        return MapReduce(compute_single_projection, combine_projections);
    }
};

}   // namespace gtfo