//----------------------------------------------------------------------------------------------------
// File: CaptureDistance.hpp
// Desc: Utility functions for jerk- and acceleration-limited capture distance
//----------------------------------------------------------------------------------------------------
#pragma once

#include <algorithm>
#include <cmath>

namespace gtfo{

template<typename Scalar = double>
struct CaptureDistanceResult{
    Scalar distance_;
    Scalar Dv_;
    Scalar Da_;
    // At max(0, excursion) == 0, Dv_/Da_ describe the excursion branch.
    // The directional derivative is max(0, Dv_*dv + Da_*da).
    bool zero_excursion_transition_;

    CaptureDistanceResult()
        :   distance_(0),
            Dv_(0),
            Da_(0),
            zero_excursion_transition_(false)
    {}

    Scalar DirectionalDerivative(const Scalar& dv, const Scalar& da) const{
        const Scalar derivative = Dv_ * dv + Da_ * da;
        return zero_excursion_transition_ ? std::max(Scalar(0), derivative) : derivative;
    }
};

template<typename Scalar = double>
CaptureDistanceResult<Scalar> ComputeCaptureDistance(
    const Scalar& velocity,
    const Scalar& acceleration,
    const Scalar& max_acceleration,
    const Scalar& max_jerk)
{
    CaptureDistanceResult<Scalar> result;

    const Scalar delta = acceleration * acceleration + 2.0 * max_jerk * velocity;
    if(delta < 0.0){
        return result;
    }

    const Scalar t_a = (acceleration + max_acceleration) / max_jerk;
    const Scalar t_v = (acceleration + std::sqrt(delta)) / max_jerk;

    if(t_v <= 0.0){
        return result;
    }

    const auto nu = [&](const Scalar& t){
        return velocity + acceleration * t - 0.5 * max_jerk * t * t;
    };

    const auto chi = [&](const Scalar& t){
        return velocity * t
            + 0.5 * acceleration * t * t
            - (1.0 / 6.0) * max_jerk * t * t * t;
    };

    if(t_v <= t_a){
        const Scalar distance = chi(t_v);
        if(distance >= 0.0){
            result.distance_ = distance;
            result.Dv_ = t_v;
            result.Da_ = 0.5 * t_v * t_v;
            result.zero_excursion_transition_ = (distance == 0.0);
        }
        return result;
    }

    const Scalar velocity_at_ta = nu(t_a);
    const Scalar distance =
        chi(t_a)
        + velocity_at_ta * velocity_at_ta / (2.0 * max_acceleration);

    if(distance >= 0.0){
        result.distance_ = distance;
        result.Dv_ = t_a + velocity_at_ta / max_acceleration;
        result.Da_ =
            0.5 * t_a * t_a
            + velocity_at_ta * t_a / max_acceleration;
        result.zero_excursion_transition_ = (distance == 0.0);
    }

    return result;
}

}   // namespace gtfo
