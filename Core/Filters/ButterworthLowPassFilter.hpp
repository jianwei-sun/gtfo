//----------------------------------------------------------------------------------------------------
// File: ButterworthLowPassFilter.hpp
// Desc: Header file for a discrete-time Butterworth low-pass filter implementation
//       Coefficients are calculated from a bilinear transform of the continuous-time low pass filter. 
//       Note that for high order filters, numerical stability of the coefficients may be a problem. 
//       For those use-cases, a higher precision floating-point type, such as double, is recommended.
//       For even higher order filters, second-order sections may be a better alternative altogether
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <array>
#include <complex>
#include <algorithm>
#include <functional>

// Project-specific includes
#include "DiscreteTimeFilter.hpp"

namespace gtfo {

template <unsigned int Dimensions, unsigned int Order, typename Scalar = double>
class ButterworthLowPassFilter : public DiscreteTimeFilter<Dimensions, Order + 1, Order + 1, Scalar>{
public:
    static_assert(Order > 0, "Filter order cannot be zero.");

    using Base = DiscreteTimeFilter<Dimensions, Order + 1, Order + 1, Scalar>;
    using Complex = std::complex<Scalar>;

    ButterworthLowPassFilter(const Scalar& sampling_frequency, const Scalar& cutoff_frequency)
        :   Base()
    {
        assert(sampling_frequency > 0.0);
        assert(0.0 < cutoff_frequency && cutoff_frequency <= sampling_frequency / 2.0);

        // Prewarp the frequency since the bilinear transform moves the desired corner frequency
        const Scalar frequency_prewarped = Scalar(2.0) * std::tan(Scalar(M_PI) * cutoff_frequency / sampling_frequency);

        // Compute the location of the continuous-time poles for a Butterworth low pass of the desired order
        std::array<Complex, Order> continuous_poles;
        for(unsigned i = 0; i < Order; ++i){
            const Scalar angle = static_cast<Scalar>(2 * i + Order + 1) * Scalar(M_PI_2) / Order;
            continuous_poles[i] = Complex(std::cos(angle), std::sin(angle)) * frequency_prewarped;
        }

        // Transform the continuous-time poles into discrete-time where the denominator of the transfer 
        // function is a product of terms (term_i - z^{-1})
        std::vector<Complex> binomial_terms;
        binomial_terms.resize(Order);
        std::transform(continuous_poles.begin(), continuous_poles.end(), binomial_terms.begin(), 
            [](const Complex& pole){
                return -(Complex(2.0) - pole) / (Complex(2.0) + pole);
            }
        );

        // Multiply out the binomial product
        const std::vector<Complex> unscaled_coefficients = BinomialProduct(binomial_terms);

        // Normalize the denominator so that the constant term is always 1, and fix the order
        typename Base::Denominator denominator = Base::Denominator::Unit(0);
        for(unsigned i = 1; i < Order + 1; ++i){
            denominator[i] = (unscaled_coefficients[Order - i] / unscaled_coefficients.back()).real(); 
        }

        // Multiply out the unscaled numerator, which is of the form (1 + z^{-1})^n
        const std::vector<int> unscaled_numerator = BinomialProduct(std::vector<int>(Order, 1));

        // Compute the numerator gain so that the overall gain is unity at dc
        const Scalar gain = static_cast<Scalar>(std::pow(frequency_prewarped, Order)) / std::transform_reduce(continuous_poles.begin(), continuous_poles.end(), 
            Complex(1.0, 0.0), 
            std::multiplies<Complex>(), 
            [](const Complex& pole){
                return Complex(2.0) - pole;
            }
        ).real();

        // Apply the gain to the numerator and fix the order
        typename Base::Numerator numerator;
        for(unsigned i = 0; i < Order + 1; ++i){
            numerator[i] = static_cast<Scalar>(unscaled_numerator[Order - i]) * gain;
        }

        // Set the coefficients
        Base::SetCoefficients(numerator, denominator);
    }

private:

    // Expands binomials of the form \prod_{i=1}^n (x+a_i), where the a_i's have template type T
    // Input: size n vector containing the a_i's
    // Output: size n+1 vector containing the coefficients of the expanded polynomial in descending powers of x
    // Note that the first output is always 1
    template<typename T>
    std::vector<T> BinomialProduct(const std::vector<T>& terms){
        // Empty case, return empty polynominal
        if(terms.empty()){
            return terms;
        }

        // Preallocate all to zero
        std::vector<T> coefficients(terms.size() + 1, T(0));

        // Copy the first binomial over
        coefficients[0] = T(1);
        coefficients[1] = terms[0];

        // For each additional binominal multiplication
        for(unsigned i = 1; i < terms.size(); ++i){
            // Increase the coefficients by the new term
            for(unsigned j = i + 1; j >= 1; --j){
                coefficients[j] += terms[i] * coefficients[j-1];
            }
        }

        return coefficients;
    }
};

}   // namespace gtfo