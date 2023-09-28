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

// Compares if two eigen types are equal; they are considered equal if the 
// infinite norm of their error is less than the specified tolerance
template <typename DerivedA, typename DerivedB>
[[nodiscard]] inline const bool IsEqual(const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b, const typename Eigen::DenseBase<DerivedA>::Scalar& tol = GTFO_EQUALITY_COMPARISON_TOLERANCE){
    return ((a.derived() - b.derived()).array().abs() <= tol).all();
}

// From: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

}   // namespace gtfo