//----------------------------------------------------------------------------------------------------
// File: Containers.hpp
// Desc: Utility class for container-related utilities
//----------------------------------------------------------------------------------------------------
#pragma once

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "Comparisons.hpp"

namespace gtfo {

// Checks to see if a STL container contains a particular vector; containment 
// refers to whether there is an element that is equal to the target vector 
// within in the standard tolerance
template <typename Container, typename VectorN>
[[nodiscard]] inline const bool ContainsVector(const Container& container, const VectorN& vector) {
    // any_of returns false if the container is empty
    return std::any_of(container.begin(), container.end(), [&vector](const VectorN& element)->bool{
        return IsEqual(element, vector);
    });
}

}   // namespace gtfo