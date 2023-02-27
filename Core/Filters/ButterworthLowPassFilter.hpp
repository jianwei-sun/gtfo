//----------------------------------------------------------------------------------------------------
// File: ButterworthLowPassFilter.hpp
// Desc: Header file for a discrete-time Butterworth low-pass filter implementation
//       Coefficients are calculated using the third-party library from Extrom Laboratory
//       (https://exstrom.com/journal/sigproc/dsigproc.html)
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <cstdlib>

// Third party library includes
#include "liir.hpp"

// Project-specific includes
#include "DiscreteTimeFilter.hpp"

namespace gtfo {

template <unsigned int Dimensions, unsigned int Order, typename Scalar = double>
class ButterworthLowPassFilter : public DiscreteTimeFilter<Dimensions, Order + 1, Order + 1, Scalar>{
public:
    static_assert(Order > 0, "Filter order cannot be zero.");

    using Base = DiscreteTimeFilter<Dimensions, Order + 1, Order + 1, Scalar>;

    ButterworthLowPassFilter(const Scalar& dt, const Scalar& cutoff_frequency)
        :   Base()
    {
        assert(dt != 0.0);
        assert(cutoff_frequency <= 1.0 / (2.0 * dt));

        // Normalize the cutoff frequency
        const double cutoff_frequency_normalized = 2.0 * cutoff_frequency * dt;

        // Get the numerator coefficients
        int* const intNumerator = ExtromLaboratories::ccof_bwlp(Order);
        const double scalingFactor = ExtromLaboratories::sf_bwlp(Order, cutoff_frequency_normalized);
        std::array<double, Order + 1> numerator;
        for(size_t i = 0; i < Order + 1; ++i){
            numerator[i] = static_cast<double>(*(intNumerator + i)) * scalingFactor;
        }

        // Get the denominator coefficients
        double* const denominator = ExtromLaboratories::dcof_bwlp(Order, cutoff_frequency_normalized);

        // Configure the filter
        Base::setCoefficients(Base::Numerator::Map(numerator.data()), Base::Denominator::Map(denominator));

        // Free the data buffers created by the third-party library
        free(intNumerator);
        free(denominator);
    }
};

}   // namespace gtfo