#include <gtest/gtest.h>

#include "../gtfo.hpp"

// Verifies that a low pass filter works as expected
TEST(FiltersTest, LowPassFilter)
{
    // 100 Hz with 25 Hz cutoff
    gtfo::ButterworthLowPassFilter<1, 4> filter(0.01, 25.0);
    std::array<double, 10> output;

    for(unsigned i = 0; i < 10; ++i){
        output[i] = filter.Step(gtfo::ButterworthLowPassFilter<1, 4>::VectorN::Ones()).value();
    }

    
}