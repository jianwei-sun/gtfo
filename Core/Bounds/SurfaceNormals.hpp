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
#include "../Utils/BooleanExpression.hpp"
#include "../Utils/Comparisons.hpp"

namespace gtfo {

template <typename VectorN>
class SurfaceNormals : public BooleanExpression<VectorN>{
public:
    using BooleanExpression = BooleanExpression<VectorN>;
    using Scalar = typename Eigen::DenseBase<VectorN>::Scalar;

    SurfaceNormals() : BooleanExpression(Relation::Intersection) {}
    SurfaceNormals(const Relation& relation) : BooleanExpression(relation) {}

    SurfaceNormals(const Relation& relation, const VectorN& surface_normal)
        :   BooleanExpression(relation, surface_normal) {}

    SurfaceNormals(const Relation& relation, const std::vector<SurfaceNormals>& surface_normals)
        :   BooleanExpression(relation, surface_normals) {}

    SurfaceNormals(const Relation& relation, const std::vector<VectorN>& surface_normals)
        :   BooleanExpression(relation, surface_normals) {}

    [[nodiscard]] bool Contains(const VectorN& vector) const{
        const std::function<bool(const VectorN&)> vector_equal = 
            [&vector](const VectorN& surface_normal)->bool{
                return IsEqual(surface_normal, vector);
            };

        const std::function<bool(const Relation&, const std::vector<bool>&)> combine_results = 
            [](const Relation& relation, const std::vector<bool>& results)->bool{
                return std::find(results.cbegin(), results.cend(), true) != results.cend();
            };

        return MapReduce(vector_equal, combine_results);
    }

    [[nodiscard]] bool IsEmpty() const{
        using VectorCollection = std::vector<VectorN>;

        const std::function<VectorCollection(const VectorN&)> identity = 
            [](const VectorN& vector)->VectorCollection{
                return VectorCollection{vector};
            };

        const std::function<VectorCollection(const Relation&, const std::vector<VectorCollection>&)> cancel_opposing_vectors = 
            [](const Relation& relation, const std::vector<VectorCollection>& collections)->VectorCollection{
                // Flatten the collection of collections into combined_vectors
                VectorCollection combined_vectors;  
                for(const VectorCollection& collection : collections){
                    std::copy(collection.cbegin(), collection.cend(), std::back_inserter(combined_vectors));
                }

                if(relation == Relation::Intersection){
                    for(typename VectorCollection::iterator i = combined_vectors.begin(); i != combined_vectors.end(); ){
                        // Check if there already is a surface normal that points in the opposite direction
                        const typename VectorCollection::iterator j = std::find_if(combined_vectors.begin(), combined_vectors.end(),
                            [&i](const VectorN& vector)->bool{
                                return IsEqual(-(*i), vector);
                            }
                        );
                        if(j != combined_vectors.end()){
                            combined_vectors.erase(j);
                            i = combined_vectors.erase(i);
                        } else{
                            ++i;
                        }
                    }
                }

                return combined_vectors;
            };

        return MapReduce(identity, cancel_opposing_vectors).empty();
    }

    [[nodiscard]] bool HasPositiveDotProductWith(const VectorN& vector) const{
        const std::function<bool(const VectorN&)> check_dot_product = 
            [&vector](const VectorN& surface_normal)->bool{
                return vector.dot(surface_normal) > 0.0;
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

        return MapReduce(check_dot_product, combine_results);
    }

    void RemoveComponentIn(VectorN& vector) const {
        // Note: I don't know how to create the logic for this function
        // Certain cases are hard to handle, such as: (a|b)&a for vectors a,b

        // if(this->relation_ == Relation::Union){
        //     for(const std::shared_ptr<VectorN>& surface_normal_ptr : this->operands_){
        //         const Scalar dot_product = surface_normal_ptr->dot(vector);
        //         if(dot_product > 0.0){
        //             vector -= dot_product * (*surface_normal_ptr);
        //         }
        //     }
        //     for(const auto& subexpression : this->subexpressions_){
        //         if(std::static_pointer_cast<SurfaceNormals>(subexpression)->HasPositiveDotProductWith(vector)){
        //             std::static_pointer_cast<SurfaceNormals>(subexpression)->RemoveComponentIn(vector);
        //         }
        //     }
        // } else{
        //     if(HasPositiveDotProductWith(vector)){
        //         VectorN negation_vector = -vector;
        //         // Negation of *this, RemoveComponentIn(negation_vector)
        //         GetNegation().RemoveComponentIn(negation_vector);
        //         vector = -negation_vector;
        //     }
        // }
    }

    [[nodiscard]] SurfaceNormals GetNegation(void) const {
        SurfaceNormals negation(this->relation_ == Relation::Intersection ? Relation::Union : Relation::Intersection);
        for(const std::shared_ptr<VectorN> surface_normal_ptr : this->operands_){
            negation.Insert(VectorN(-(*surface_normal_ptr)));
        }
        for(const std::shared_ptr<BooleanExpression>& subexpression : this->subexpressions_){
            negation.Insert(std::static_pointer_cast<SurfaceNormals>(subexpression)->GetNegation());
        }
        return negation;
    }
};

}   // namespace gtfo