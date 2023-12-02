//----------------------------------------------------------------------------------------------------
// File: UtiiltyFunctions.hpp
// Desc: Utility class for math and simple functions
//----------------------------------------------------------------------------------------------------
#pragma once

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "Constants.hpp"

namespace gtfo {

template<typename Scalar> 
int sgn(const Scalar& x) {
    return (Scalar(0) < x) - (x < Scalar(0));
}

// Implements sin(x)/x by using a Taylor expansion near the origin
template<typename Scalar>
Scalar sinc(const Scalar& x){
    if(std::abs(x) < GTFO_EQUALITY_COMPARISON_TOLERANCE){
        return 1.0 - x * x / 6.0;
    } else{
        return std::sin(x) / x;
    }
}

}   // namespace gtfo