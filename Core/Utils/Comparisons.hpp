//----------------------------------------------------------------------------------------------------
// File: Comparisons.hpp
// Desc: Utility class for comparison of matrix objects
//----------------------------------------------------------------------------------------------------
#pragma once

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "Constants.hpp"

namespace gtfo {

template <typename DerivedA, typename DerivedB>
[[nodiscard]] inline const bool IsEqual(const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b, const typename Eigen::DenseBase<DerivedA>::Scalar& tol = GTFO_EQUALITY_COMPARISON_TOLERANCE){
    return ((a.derived() - b.derived()).array().abs() <= tol).all();
}

}   // namespace gtfo