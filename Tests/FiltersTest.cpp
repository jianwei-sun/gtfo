#include <gtest/gtest.h>
#include "gtfo.hpp"

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
        EXPECT_NEAR(output[i], expected_output[i], 1e-14);
    }
}

// Verifies a few coefficients by comparing it to Matlab's butter
TEST(FiltersTest, LowPassFilterCoefficients1)
{
    gtfo::ButterworthLowPassFilter<1, 2> filter(10.0, 3.0);
    const Eigen::Matrix<double, 3, 1> expected_numerator(
        0.391335772501769,
        0.782671545003537,
        0.391335772501769
    );
    const Eigen::Matrix<double, 3, 1> expected_denominator(
        1.000000000000000,
        0.369527377351241,
        0.195815712655833
    );
    EXPECT_TRUE(gtfo::IsEqual(expected_numerator, filter.GetNumerator(), 1e-14));
    std::cout << filter.GetNumerator().transpose() << std::endl;
    EXPECT_TRUE(gtfo::IsEqual(expected_denominator, filter.GetDenominator(), 1e-14));
    std::cout << filter.GetDenominator().transpose() << std::endl;
}

TEST(FiltersTest, LowPassFilterCoefficients2)
{
    gtfo::ButterworthLowPassFilter<2, 4, float> filter(100.0, 25.0);
    const Eigen::Matrix<float, 5, 1> expected_numerator(
        0.093980851433795f,
        0.375923405735178f,
        0.563885108602767f,
        0.375923405735178f,
        0.093980851433795f
    );
    const Eigen::Matrix<float, 5, 1> expected_denominator(
        1.000000000000000f,
        0.000000000000001f,
        0.486028822068270f,
        0.000000000000000f,
        0.017664800872442f
    );
    // A coarser tolerance is needed here since the underlying floating-type is float
    // Recommended to use at least double precision
    EXPECT_TRUE(gtfo::IsEqual(expected_numerator, filter.GetNumerator(), 1e-6f));
    std::cout << filter.GetNumerator().transpose() << std::endl;
    EXPECT_TRUE(gtfo::IsEqual(expected_denominator, filter.GetDenominator(), 1e-6f));
    std::cout << filter.GetDenominator().transpose() << std::endl;
	std::cout << expected_denominator.transpose() << std::endl;
	std::cout << (filter.GetDenominator() - expected_denominator).norm() << std::endl;
}

TEST(FiltersTest, LowPassFilterCoefficients3)
{
    gtfo::ButterworthLowPassFilter<1, 6> filter(1000.0, 235.0);
    const Eigen::Matrix<double, 7, 1> expected_numerator(
        0.022105040504827,
        0.132630243028961,
        0.331575607572401,
        0.442100810096535,
        0.331575607572401,
        0.132630243028961,
        0.022105040504827
    );
    const Eigen::Matrix<double, 7, 1> expected_denominator(
        1.000000000000000,
        -0.356061427933937,
        0.825015917881444,
        -0.171171303737869,
        0.126556382247908,
        -0.011622264283415,
        0.002005288134782
    );
    EXPECT_TRUE(gtfo::IsEqual(expected_numerator, filter.GetNumerator(), 1e-14));
    std::cout << filter.GetNumerator().transpose() << std::endl;
    EXPECT_TRUE(gtfo::IsEqual(expected_denominator, filter.GetDenominator(), 1e-14));
    std::cout << filter.GetDenominator().transpose() << std::endl;
}

// Verifies that a rate limiter works as expected
TEST(FiltersTest, RateLimiter)
{
    gtfo::RateLimiter<2> rate_limiter(1.0, Eigen::Vector2d::Ones());
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
