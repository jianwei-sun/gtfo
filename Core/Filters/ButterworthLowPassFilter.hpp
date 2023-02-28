//----------------------------------------------------------------------------------------------------
// File: ButterworthLowPassFilter.hpp
// Desc: Header file for a discrete-time Butterworth low-pass filter implementation
//       Coefficients are calculated using the third-party library from Exstrom Laboratory
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

    ButterworthLowPassFilter(const Scalar& sampling_frequency, const Scalar& cutoff_frequency)
        :   Base()
    {
        assert(sampling_frequency != 0.0);
        assert(cutoff_frequency <= sampling_frequency / 2.0);

        // Normalize the cutoff frequency
        const double cutoff_frequency_normalized = 2.0 * cutoff_frequency / sampling_frequency;

        // Get the numerator coefficients
        int* const intNumerator = ExstromLaboratories::ccof_bwlp(Order);
        const double scalingFactor = ExstromLaboratories::sf_bwlp(Order, cutoff_frequency_normalized);
        std::array<double, Order + 1> numerator;
        for(size_t i = 0; i < Order + 1; ++i){
            numerator[i] = static_cast<double>(*(intNumerator + i)) * scalingFactor;
        }

        // Get the denominator coefficients
        double* const denominator = ExstromLaboratories::dcof_bwlp(Order, cutoff_frequency_normalized);

        // Configure the filter
        Base::SetCoefficients(Base::Numerator::Map(numerator.data()), Base::Denominator::Map(denominator));

        // Free the data buffers created by the third-party library
        free(intNumerator);
        free(denominator);
    }
};

}   // namespace gtfo