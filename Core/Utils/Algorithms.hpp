//----------------------------------------------------------------------------------------------------
// File: Algorithms.hpp
// Desc: Utility class for common algorithms
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <functional>

// Project-specific
#include "Constants.hpp"

namespace gtfo {

// Performs a bisection search
template <typename Scalar>
[[nodiscard]] Scalar BisectionSearch(std::function<bool(const Scalar&)> evaluator, const Scalar& tol = GTFO_ALGORITHMIC_CONVERGENCE_TOLERANCE) {
    static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");
    
    std::pair<Scalar, Scalar> search_interval = std::make_pair(0.0, 1.0);
    while((search_interval.second - search_interval.first) > tol){
        const Scalar scale = (search_interval.first + search_interval.second) / 2.0;
        if(evaluator(scale)){
            search_interval.first = scale;
        } else{
            search_interval.second = scale;
        }
    }
    return search_interval.first;
}

}   // namespace gtfo