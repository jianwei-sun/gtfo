#include <gtest/gtest.h>

#include "../gtfo.hpp"

// Verifies that a low pass filter works as expected
TEST(FiltersTest, LowPassFilter)
{
    // 4th Order low-pass filter with 100 Hz sampling and 25 Hz cutoff
    gtfo::ButterworthLowPassFilter<1, 4> filter(100.0, 25.0);
    std::array<double, 10> output;

    for(unsigned i = 0; i < 10; ++i){
        output[i] = filter.Step(gtfo::ButterworthLowPassFilter<1, 4>::VectorN::Ones()).value();
        std::cout << output[i] << "\n";
    }

    // Compare with expected output
    const std::array<double, 10> expected_output{
        0.093980851433795,
        0.469904257168973,
        0.988111963252399,
        1.181325758910216,
        1.021782576343182,
        0.921234490726671,
        0.989623039880221,
        1.035079224267530,
        1.004658716830074,
        0.984341862987113
    };

    for(unsigned i = 0; i < 10; ++i){
        EXPECT_NEAR(output[i], expected_output[i], 1e-15);
    }
}

// Verifies that a rate limiter works as expected
TEST(FiltersTest, RateLimiter)
{
    gtfo::RateLimiter<2> rate_limiter(1.0, 1.0);
    Eigen::Vector2d output = Eigen::Vector2d::Zero();

    for(unsigned i = 1; i <= 5; ++i){
        output = rate_limiter.Step(Eigen::Vector2d(static_cast<double>(i), 10.0));
        EXPECT_NEAR(output[0], static_cast<double>(i), 1e-15);
        EXPECT_NEAR(output[0], output[1], 1e-15);
    }

    for(unsigned i = 1; i <= 5; ++i){
        output = rate_limiter.Step(Eigen::Vector2d(5.0, 10.0));
        EXPECT_NEAR(output[0], 5.0, 1e-15);
        EXPECT_NEAR(output[1], static_cast<double>(i + 5), 1e-15);
    }

    for(unsigned i = 0; i < 5; ++i){
        output = rate_limiter.Step(Eigen::Vector2d(5.0, 10.0));
        EXPECT_NEAR(output[0], 5.0, 1e-15);
        EXPECT_NEAR(output[1], 10.0, 1e-15);
    }
}