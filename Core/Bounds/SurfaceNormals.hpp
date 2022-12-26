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

#include <iostream>

namespace gtfo {

template <typename VectorType>
class SurfaceNormals : public BooleanExpression<VectorType>{
public:
    using BooleanExpression = BooleanExpression<VectorType>;
    using Scalar = typename Eigen::DenseBase<VectorType>::Scalar;

    SurfaceNormals() : BooleanExpression(Relation::Intersection) {}

    template <typename... InputTypes>
    SurfaceNormals(const Relation& relation, const InputTypes&... inputs) 
        : BooleanExpression(relation, inputs...) {
            std::cout << "Created SurfaceNormals with ";
            if(relation == Relation::Intersection){
                std::cout << "Intersection";
            } else{
                std::cout << "Union";
            }
            std::cout << ", and " << this->operands_.size() << " operands and " 
            << this->subexpressions_.size() << " subexpressions.\n";
        }

    // SurfaceNormals(const Relation& relation, const std::vector<SurfaceNormals>& collection)
    //     : BooleanExpression(relation, collection) {}

    // SurfaceNormals(const Relation& relation, const std::vector<VectorType>& vectors)
    //     : BooleanExpression(relation, vectors) {}

    [[nodiscard]] bool Contains(const VectorType& vector) const{
        const std::function<bool(const VectorType&)> vector_equal = 
            [&vector](const VectorType& surface_normal)->bool{
                return IsEqual(surface_normal, vector);
            };

        const std::function<bool(const Relation&, const std::vector<bool>&)> combine_results = 
            [](const Relation& relation, const std::vector<bool>& results)->bool{
                // if(relation == Relation::Union){
                //     // True if any is true
                //     return std::find(results.cbegin(), results.cend(), true) != results.cend();
                // } else{
                //     // True if all is true
                //     return std::find(results.cbegin(), results.cend(), false) == results.cend();
                // }
                return std::find(results.cbegin(), results.cend(), true) != results.cend();
            };

        return MapReduce(vector_equal, combine_results);
    }

    [[nodiscard]] bool IsEmpty() const{
        using VectorCollection = std::vector<VectorType>;

        const std::function<VectorCollection(const VectorType&)> identity = 
            [](const VectorType& vector)->VectorCollection{
                return VectorCollection{vector};
            };

        const std::function<VectorCollection(const Relation&, const std::vector<VectorCollection>&)> cancel_opposing_vectors = 
            [](const Relation& relation, const std::vector<VectorCollection>& collections)->VectorCollection{
                // Flatten the collection of collections into combined_vectors
                VectorCollection combined_vectors;  
                for(const VectorCollection& collection : collections){
                    std::copy(collection.cbegin(), collection.cend(), std::back_inserter(combined_vectors));
                }

                std::cout << "combined_vectors has " << combined_vectors.size() << " members.\n";

                if(relation == Relation::Intersection){
                    for(typename VectorCollection::iterator i = combined_vectors.begin(); i != combined_vectors.end(); ){
                        // Check if there already is a surface normal that points in the opposite direction
                        const typename VectorCollection::iterator j = std::find_if(combined_vectors.begin(), combined_vectors.end(),
                            [&i](const VectorType& vector)->bool{
                                return IsEqual(-(*i), vector);
                            }
                        );
                        if(j != combined_vectors.end()){
                            std::cout << "Erased something!\n";
                            combined_vectors.erase(j);
                            i = combined_vectors.erase(i);
                        } else{
                            ++i;
                        }
                    }
                }

                return combined_vectors;

                // if(relation == Relation::Intersection){
                //     for(const VectorType& vector : combined_vectors){
                //         // Check if there already is a surface normal that points in the opposite direction
                //         const typename std::vector<VectorType>::iterator it = std::find_if(combined_vectors.begin(), combined_vectors.end(), [&vector](const VectorType& combined_vector)->bool{
                //             return IsEqual(-vector, combined_vector);
                //         });

                //         // If there is, remove it
                //         if(it != combined_vectors.end()){
                //             combined_vectors.erase(it);
                //         // Otherwise, insert the surface normal to the collection
                //         } else{
                //             combined_vectors.push_back(vector);
                //         }
                //     }
                    
                //     // If the combined_vectors is empty, or if it only contains zero vectors, then return a zero vector to indicate empty
                //     if(std::all_of(combined_vectors.cbegin(), combined_vectors.cend(), 
                //         [](const VectorType& vector)->bool{
                //             return vector.isZero();
                //         })
                //     ){
                //         return VectorType::Zero();
                //     }
                // } else{
                //     // Similarly, in the union case, if the input vectors are empty or only contains zero vectors, then return zero
                //     if(std::all_of(vectors.cbegin(), vectors.cend(), 
                //         [](const VectorType& vector)->bool{
                //             return vector.isZero();
                //         })
                //     ){
                //         return VectorType::Zero();
                //     }
                // }

                // // At this point, it means there are vectors that do not cancel out, so do not return a zero vector
                // return VectorType::Ones();
            };

        return MapReduce(identity, cancel_opposing_vectors).empty();
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