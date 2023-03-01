//----------------------------------------------------------------------------------------------------
// File: RateLimiter.hpp
// Desc: Header file for a rate-limiter filter for use in physical human-robot interaction
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <type_traits>

// Third party library includes
#include <Eigen/Dense>

namespace gtfo {

template <unsigned int Dimensions, typename Scalar = double>
class RateLimiter{
public:
    static_assert(Dimensions >= 1, "Dimension must be at least 1.");
    
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    RateLimiter(const Scalar& sampling_frequency, const Scalar& max_rate, const VectorN& initial_state = VectorN::Zero())
        :   max_difference_(max_rate / sampling_frequency),
            state_(initial_state) {}

    VectorN Step(const VectorN& input){
        // Limit the maximum difference between the input and state
        state_ += (input - state_).cwiseMax(-max_difference_).cwiseMin(max_difference_);
        return state_;
    }

private:
    const Scalar max_difference_;
    VectorN state_;
};

}   // namespace gtfo