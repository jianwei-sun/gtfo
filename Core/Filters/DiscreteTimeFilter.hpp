//----------------------------------------------------------------------------------------------------
// File: DiscreteTimeFilter.hpp
// Desc: Header file for a discrete time filter based on the Direct Form II implementation. 
//       Implements a custom discrete time filter of the form:
//
//          numerator[0] + numerator[1]z^-1 + ... + numerator[M]z^-M
//       --------------------------------------------------------------
//       denominator[0] + denominator[1]z^-1 + ... + denominator[N]z^-N
//
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <array>
#include <cassert>
#include <limits>
#include <numeric>
#include <type_traits>
#include <algorithm>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "../Utils/CircularBuffer.hpp"

namespace gtfo {

template <unsigned int Dimensions, unsigned int NumeratorDimension, unsigned int DenominatorDimension, typename Scalar = double>
class DiscreteTimeFilter{
public:
    static_assert(std::is_floating_point<Scalar>::value, "Scalar must be a floating-point type");
    static_assert(NumeratorDimension > 0, "Numerator length cannot be zero");
    static_assert(DenominatorDimension > 0, "Denominator length cannot be zero");

    using Numerator = Eigen::Matrix<Scalar, NumeratorDimension, 1>;
    using Denominator = Eigen::Matrix<Scalar, DenominatorDimension, 1>;
    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;

    // A default initialization sets numerator[0] and denominator[0] to unity, which makes the filter simply be output = input
    DiscreteTimeFilter()
        :   numerator_(Numerator::Unit(0)),
            denominator_(Denominator::Unit(0))
    {}

    DiscreteTimeFilter(const Numerator& numerator, const Denominator& denominator){
        SetCoefficients(numerator, denominator);
    }

    void SetCoefficients(const Numerator& numerator, const Denominator& denominator){
        assert(fabs(denominator[0] - 1.0) <= std::numeric_limits<Scalar>::epsilon());
        numerator_ = numerator;
        denominator_ = denominator;

        for(size_t i = 0; i < (std::max)(NumeratorDimension, DenominatorDimension); ++i){
            delayed_states_[i].setZero();
        }
    }

    Numerator GetNumerator(void) const{
        return numerator_;
    }

    Denominator GetDenominator(void) const{
        return denominator_;
    }

    VectorN Step(const VectorN& input){
        VectorN internal = input;
        for(size_t i = 1; i < DenominatorDimension; ++i){
            internal -= delayed_states_[i-1] * denominator_[i];
        }

        VectorN output = internal * numerator_[0];
        for(size_t i = 1; i < NumeratorDimension; ++i){
            output += delayed_states_[i-1] * numerator_[i];
        }

        delayed_states_.push_front(internal);
        return output;
    }

private:
    Numerator numerator_;
    Denominator denominator_;

    CircularBuffer<VectorN, (std::max)(NumeratorDimension, DenominatorDimension)> delayed_states_;
} ;

}   // namespace gtfo