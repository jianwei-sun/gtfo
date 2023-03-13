//----------------------------------------------------------------------------------------------------
// File: RateLimiter.hpp
// Desc: Header file for a rate-limiter filter for use in physical human-robot interaction based on
//       J. Sun, P. W. Ferguson and J. Rosen, "Suppressing Delay-Induced Oscillations in 
//       Physical Human-Robot Interaction with an Upper-Limb Exoskeleton using Rate-Limiting," 
//       2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2022, 
//       pp. 6695-6701, doi: 10.1109/IROS47612.2022.9981943.
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

    RateLimiter(const Scalar& sampling_frequency, const VectorN& max_rates, const VectorN& initial_state = VectorN::Zero())
        :   max_differences_(max_rates / sampling_frequency),
            state_(initial_state) 
    {
        assert(sampling_frequency > 0.0);
        assert((max_rates.array() >= 0.0).all());
    }

    VectorN Step(const VectorN& input){
        // Limit the maximum difference between the input and state
        state_ += (input - state_).cwiseMax(-max_differences_).cwiseMin(max_differences_);
        return state_;
    }

private:
    const VectorN max_differences_;
    VectorN state_;
};

}   // namespace gtfo