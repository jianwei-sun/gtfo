#include <gtest/gtest.h>

#include "../gtfo.hpp"

TEST(NormBoundTest, Contains1D) {
    const std::array<double, 5> thresholds = {0.1, 0.5, 1.0, 2.0, 10.0};

    // 1-D 
    Eigen::Matrix<double, 1, 1> test_point_1d(-1.0);
    // 1-D Infinite-Norm
    EXPECT_FALSE((gtfo::NormBound<Eigen::Infinity, 1>(thresholds[0]).Contains(test_point_1d)));
    EXPECT_FALSE((gtfo::NormBound<Eigen::Infinity, 1>(thresholds[1]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 1>(thresholds[2]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 1>(thresholds[3]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 1>(thresholds[4]).Contains(test_point_1d)));
    // 1-D 1-Norm
    EXPECT_FALSE((gtfo::NormBound<1, 1>(thresholds[0]).Contains(test_point_1d)));
    EXPECT_FALSE((gtfo::NormBound<1, 1>(thresholds[1]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<1, 1>(thresholds[2]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<1, 1>(thresholds[3]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<1, 1>(thresholds[4]).Contains(test_point_1d)));
    // 1-D 2-Norm
    EXPECT_FALSE((gtfo::NormBound<2, 1>(thresholds[0]).Contains(test_point_1d)));
    EXPECT_FALSE((gtfo::NormBound<2, 1>(thresholds[1]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<2, 1>(thresholds[2]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<2, 1>(thresholds[3]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<2, 1>(thresholds[4]).Contains(test_point_1d)));
    // 1-D 3-Norm
    EXPECT_FALSE((gtfo::NormBound<3, 1>(thresholds[0]).Contains(test_point_1d)));
    EXPECT_FALSE((gtfo::NormBound<3, 1>(thresholds[1]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<3, 1>(thresholds[2]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<3, 1>(thresholds[3]).Contains(test_point_1d)));
    EXPECT_TRUE((gtfo::NormBound<3, 1>(thresholds[4]).Contains(test_point_1d)));
}

TEST(NormBoundTest, Contains2D) {
    const std::array<double, 5> thresholds = {0.1, 0.5, 1.0, 2.0, 10.0};
    // 2-D 
    Eigen::Matrix<double, 2, 1> test_point_2d(-1.0, 0.0);
    // 2-D Infinite-Norm
    EXPECT_FALSE((gtfo::NormBound<Eigen::Infinity, 2>(thresholds[0]).Contains(test_point_2d)));
    EXPECT_FALSE((gtfo::NormBound<Eigen::Infinity, 2>(thresholds[1]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 2>(thresholds[2]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 2>(thresholds[3]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 2>(thresholds[4]).Contains(test_point_2d)));
    // 2-D 1-Norm
    EXPECT_FALSE((gtfo::NormBound<1, 2>(thresholds[0]).Contains(test_point_2d)));
    EXPECT_FALSE((gtfo::NormBound<1, 2>(thresholds[1]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<1, 2>(thresholds[2]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<1, 2>(thresholds[3]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<1, 2>(thresholds[4]).Contains(test_point_2d)));
    // 2-D 2-Norm
    EXPECT_FALSE((gtfo::NormBound<2, 2>(thresholds[0]).Contains(test_point_2d)));
    EXPECT_FALSE((gtfo::NormBound<2, 2>(thresholds[1]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<2, 2>(thresholds[2]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<2, 2>(thresholds[3]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<2, 2>(thresholds[4]).Contains(test_point_2d)));
    // 2-D 3-Norm
    EXPECT_FALSE((gtfo::NormBound<3, 2>(thresholds[0]).Contains(test_point_2d)));
    EXPECT_FALSE((gtfo::NormBound<3, 2>(thresholds[1]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<3, 2>(thresholds[2]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<3, 2>(thresholds[3]).Contains(test_point_2d)));
    EXPECT_TRUE((gtfo::NormBound<3, 2>(thresholds[4]).Contains(test_point_2d)));
}

TEST(NormBoundTest, Contains3D) {
    const std::array<double, 5> thresholds = {0.1, 0.5, 1.0, 2.0, 10.0};
    // 3-D 
    Eigen::Matrix<double, 3, 1> test_point_3d(-1.0, 0.0, 0.0);
    // 3-D Infinite-Norm
    EXPECT_FALSE((gtfo::NormBound<Eigen::Infinity, 3>(thresholds[0]).Contains(test_point_3d)));
    EXPECT_FALSE((gtfo::NormBound<Eigen::Infinity, 3>(thresholds[1]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 3>(thresholds[2]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 3>(thresholds[3]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<Eigen::Infinity, 3>(thresholds[4]).Contains(test_point_3d)));
    // 3-D 1-Norm
    EXPECT_FALSE((gtfo::NormBound<1, 3>(thresholds[0]).Contains(test_point_3d)));
    EXPECT_FALSE((gtfo::NormBound<1, 3>(thresholds[1]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<1, 3>(thresholds[2]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<1, 3>(thresholds[3]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<1, 3>(thresholds[4]).Contains(test_point_3d)));
    // 3-D 2-Norm
    EXPECT_FALSE((gtfo::NormBound<2, 3>(thresholds[0]).Contains(test_point_3d)));
    EXPECT_FALSE((gtfo::NormBound<2, 3>(thresholds[1]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<2, 3>(thresholds[2]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<2, 3>(thresholds[3]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<2, 3>(thresholds[4]).Contains(test_point_3d)));
    // 3-D 3-Norm
    EXPECT_FALSE((gtfo::NormBound<3, 3>(thresholds[0]).Contains(test_point_3d)));
    EXPECT_FALSE((gtfo::NormBound<3, 3>(thresholds[1]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<3, 3>(thresholds[2]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<3, 3>(thresholds[3]).Contains(test_point_3d)));
    EXPECT_TRUE((gtfo::NormBound<3, 3>(thresholds[4]).Contains(test_point_3d)));
}

TEST(NormBoundTest, NearestPointWithinBound1D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 1> boundInfNorm(threshold);
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(-0.5))));

    // 1-Norm
    gtfo::NormBound<1, 1> bound1Norm(threshold);
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(-0.5))));

    // 2-Norm
    gtfo::NormBound<2, 1> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(-0.5))));

    // 3-Norm
    gtfo::NormBound<3, 1> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
}

TEST(NormBoundTest, NearestPointWithinBound2D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 2> boundInfNorm(threshold);
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(0.4, 0.3))));
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 0.2), Eigen::Vector2d(-0.4, 0.3))));
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Vector2d(-2.0, -2.0), Eigen::Vector2d(-1.0, -1.0))));

    // 1-Norm
    gtfo::NormBound<1, 2> bound1Norm(threshold);
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(0.2, 0.2))));
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(0.0, 0.0))));
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Vector2d(-10.0, 20.0), Eigen::Vector2d(0.1, -0.2))));

    // 2-Norm
    gtfo::NormBound<2, 2> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(0.2, 0.2))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector2d(10.0, 10.0), Eigen::Vector2d(0.0, 0.0))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector2d(10.0, 0.1), Eigen::Vector2d(-0.8, -0.1))));

    // 3-Norm
    gtfo::NormBound<3, 2> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(0.2, 0.2))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector2d(10.0, 10.0), Eigen::Vector2d(0.0, 0.0))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector2d(10.0, 0.1), Eigen::Vector2d(-0.8, -0.1))));
}

TEST(NormBoundTest, NearestPointWithinBound3D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 3> boundInfNorm(threshold);
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Vector3d(0.4, 0.3, 0.2))));
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Vector3d(-12.0, 12.0, -5.0), Eigen::Vector3d(0.3, -0.2, 0.5))));
    EXPECT_TRUE(boundInfNorm.Contains(boundInfNorm.GetNearestPointWithinBound(Eigen::Vector3d(20.0, 20.0, 20.0), Eigen::Vector3d(-1.0, -1.0, -1.0))));

    // 1-Norm
    gtfo::NormBound<1, 3> bound1Norm(threshold);
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(0.2, 0.2, 0.2))));
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Vector3d(-10.0, 0.0, 10.0), Eigen::Vector3d(-0.3, 0.5, 0.0))));
    EXPECT_TRUE(bound1Norm.Contains(bound1Norm.GetNearestPointWithinBound(Eigen::Vector3d(10.0, 10.0, 10.0), Eigen::Vector3d(0.0, 0.0, 0.0))));

    // 2-Norm
    gtfo::NormBound<2, 3> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector3d(10.0, 10.0, 10.0), Eigen::Vector3d(0.0, 0.0, 0.0))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector3d(5.0, -3.0, 2.0), Eigen::Vector3d(-0.3, 0.4, 0.1))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector3d(0.0, 100.0, 0.0), Eigen::Vector3d(0.2, -0.3, 0.2))));
    
    // 3-Norm
    gtfo::NormBound<3, 3> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector3d(10.0, 10.0, 10.0), Eigen::Vector3d(0.0, 0.0, 0.0))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector3d(5.0, -3.0, 2.0), Eigen::Vector3d(-0.3, 0.4, 0.1))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector3d(0.0, 100.0, 0.0), Eigen::Vector3d(0.2, -0.3, 0.2))));
}

TEST(NormBoundTest, IsAtBoundary1D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 1> boundInfNorm(threshold);
    EXPECT_TRUE(boundInfNorm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(boundInfNorm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(-1.0)));
    EXPECT_FALSE(boundInfNorm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(0.0)));
    EXPECT_FALSE(boundInfNorm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(2.0)));

    // 1-Norm
    gtfo::NormBound<1, 1> bound1Norm(threshold);
    EXPECT_TRUE(bound1Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(bound1Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(-1.0)));
    EXPECT_FALSE(bound1Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(0.0)));
    EXPECT_FALSE(bound1Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(2.0)));

    // 2-Norm
    gtfo::NormBound<2, 1> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(-1.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(0.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(2.0)));

    // 3-Norm
    gtfo::NormBound<3, 1> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(-1.0)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(0.0)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(2.0)));
}

TEST(NormBoundTest, IsAtBoundary2D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 2> boundInfNorm(threshold);
    EXPECT_TRUE(boundInfNorm.IsAtBoundary(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(boundInfNorm.IsAtBoundary(Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(boundInfNorm.IsAtBoundary(Eigen::Vector2d(-1.0, 1.0)));
    EXPECT_FALSE(boundInfNorm.IsAtBoundary(Eigen::Vector2d(2.0, 1.0)));
    EXPECT_FALSE(boundInfNorm.IsAtBoundary(Eigen::Vector2d(0.0, -1.2)));
    EXPECT_FALSE(boundInfNorm.IsAtBoundary(Eigen::Vector2d(-1.0, 1.2)));

    // 1-Norm
    gtfo::NormBound<1, 2> bound1Norm(threshold);
    EXPECT_TRUE(bound1Norm.IsAtBoundary(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(bound1Norm.IsAtBoundary(Eigen::Vector2d(-0.5, -0.5)));
    EXPECT_TRUE(bound1Norm.IsAtBoundary(Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_FALSE(bound1Norm.IsAtBoundary(Eigen::Vector2d(1.0, 1.0)));
    EXPECT_FALSE(bound1Norm.IsAtBoundary(Eigen::Vector2d(-1.0, 0.1)));

    // 2-Norm
    gtfo::NormBound<2, 2> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Vector2d(std::sqrt(2.0)/2.0, -std::sqrt(2.0)/2.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Vector2d(1.0, 0.1)));

    // 3-Norm
    gtfo::NormBound<3, 2> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Vector2d(0.3, 0.9909)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Vector2d(0.3, 1.0)));
}

TEST(NormBoundTest, IsAtBoundary3D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 3> boundInfNorm(threshold);
    EXPECT_TRUE(boundInfNorm.IsAtBoundary(Eigen::Vector3d(1.0, 0.0, -1.0)));
    EXPECT_TRUE(boundInfNorm.IsAtBoundary(Eigen::Vector3d(-1.0, -1.0, -1.0)));
    EXPECT_TRUE(boundInfNorm.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 1.0)));
    EXPECT_FALSE(boundInfNorm.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 0.9)));
    EXPECT_FALSE(boundInfNorm.IsAtBoundary(Eigen::Vector3d(1.0, 1.0, 1.1)));

    // 1-Norm
    gtfo::NormBound<1, 3> bound1Norm(threshold);
    EXPECT_TRUE(bound1Norm.IsAtBoundary(Eigen::Vector3d(-0.3333, 0.3333, 0.3333)));
    EXPECT_TRUE(bound1Norm.IsAtBoundary(Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_FALSE(bound1Norm.IsAtBoundary(Eigen::Vector3d(1.0, 1.0, 1.0)));
    EXPECT_FALSE(bound1Norm.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 0.9)));

    // 2-Norm
    gtfo::NormBound<2, 3> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Vector3d(0.0, -1.0, 0.0)));
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Vector3d(std::sqrt(3.0)/3.0, -std::sqrt(3.0)/3.0, std::sqrt(3.0)/3.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Vector3d(1.0, 1.0, 1.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 0.9)));

    // 3-Norm
    gtfo::NormBound<3, 3> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Vector3d(0.6934, -0.6934, -0.6934)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Vector3d(1.0, 1.0, 1.0)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 0.9)));
}

TEST(NormBoundTest, GetSurfaceNormal1D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 1> boundInfNorm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Matrix<double, 1, 1>(1.0)), Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Matrix<double, 1, 1>(-1.0)), Eigen::Matrix<double, 1, 1>(-1.0)));

    // 1-Norm
    gtfo::NormBound<1, 1> bound1Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound1Norm.GetSurfaceNormal(Eigen::Matrix<double, 1, 1>(1.0)), Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound1Norm.GetSurfaceNormal(Eigen::Matrix<double, 1, 1>(-1.0)), Eigen::Matrix<double, 1, 1>(-1.0)));

    // 2-Norm
    gtfo::NormBound<2, 1> bound2Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound2Norm.GetSurfaceNormal(Eigen::Matrix<double, 1, 1>(1.0)), Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound2Norm.GetSurfaceNormal(Eigen::Matrix<double, 1, 1>(-1.0)), Eigen::Matrix<double, 1, 1>(-1.0)));

    // 3-Norm
    gtfo::NormBound<3, 1> bound3Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound3Norm.GetSurfaceNormal(Eigen::Matrix<double, 1, 1>(1.0)), Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound3Norm.GetSurfaceNormal(Eigen::Matrix<double, 1, 1>(-1.0)), Eigen::Matrix<double, 1, 1>(-1.0)));
}

TEST(NormBoundTest, GetSurfaceNormal2D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 2> boundInfNorm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Vector2d(0.0, 1.0)), Eigen::Vector2d(0.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Vector2d(-1.0, 0.0)), Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Vector2d(1.0, 0.99)), Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Vector2d(1.0, -1.0)), Eigen::Vector2d(1.0, -1.0).normalized()));

    // 1-Norm
    gtfo::NormBound<1, 2> bound1Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound1Norm.GetSurfaceNormal(Eigen::Vector2d(0.0, 1.0)), Eigen::Vector2d(0.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound1Norm.GetSurfaceNormal(Eigen::Vector2d(0.5, 0.5)), Eigen::Vector2d(1.0, 1.0).normalized()));
    EXPECT_TRUE(gtfo::IsEqual(bound1Norm.GetSurfaceNormal(Eigen::Vector2d(0.7, 0.3)), Eigen::Vector2d(1.0, 1.0).normalized()));

    // 2-Norm
    gtfo::NormBound<2, 2> bound2Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound2Norm.GetSurfaceNormal(Eigen::Vector2d(0.0, 1.0)), Eigen::Vector2d(0.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound2Norm.GetSurfaceNormal(Eigen::Vector2d(1.0, 1.0).normalized()), Eigen::Vector2d(1.0, 1.0).normalized()));
    EXPECT_TRUE(gtfo::IsEqual(bound2Norm.GetSurfaceNormal(Eigen::Vector2d(-1.0, 0.0)), Eigen::Vector2d(-1.0, 0.0)));
    
    // 3-Norm
    gtfo::NormBound<3, 2> bound3Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound3Norm.GetSurfaceNormal(Eigen::Vector2d(0.0, 1.0)), Eigen::Vector2d(0.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound3Norm.GetSurfaceNormal(Eigen::Vector2d(0.7937, 0.7937)), Eigen::Vector2d(1.0, 1.0).normalized()));
    EXPECT_TRUE(gtfo::IsEqual(bound3Norm.GetSurfaceNormal(Eigen::Vector2d(-1.0, 0.0)), Eigen::Vector2d(-1.0, 0.0)));
}

TEST(NormBoundTest, GetSurfaceNormal3D) {
    static constexpr double threshold = 1.0;

    // Infinite-Norm
    gtfo::NormBound<Eigen::Infinity, 3> boundInfNorm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Vector3d(0.0, 0.0, 1.0)), Eigen::Vector3d(0.0, 0.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Vector3d(-1.0, 0.5, -1.0)), Eigen::Vector3d(-1.0, 0.0, -1.0).normalized()));
    EXPECT_TRUE(gtfo::IsEqual(boundInfNorm.GetSurfaceNormal(Eigen::Vector3d(1.0, -1.0, 1.0)), Eigen::Vector3d(1.0, -1.0, 1.0).normalized()));

    // 1-Norm
    gtfo::NormBound<1, 3> bound1Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound1Norm.GetSurfaceNormal(Eigen::Vector3d(0.0, 0.0, 1.0)), Eigen::Vector3d(0.0, 0.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound1Norm.GetSurfaceNormal(Eigen::Vector3d(0.3333, -0.3333, 0.3333)), Eigen::Vector3d(1.0, -1.0, 1.0).normalized()));

    // 2-Norm
    gtfo::NormBound<2, 3> bound2Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound2Norm.GetSurfaceNormal(Eigen::Vector3d(0.0, 0.0, 1.0)), Eigen::Vector3d(0.0, 0.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound2Norm.GetSurfaceNormal(Eigen::Vector3d(1.0, 1.0, 1.0).normalized()), Eigen::Vector3d(1.0, 1.0, 1.0).normalized()));

    // 3-Norm
    gtfo::NormBound<3, 3> bound3Norm(threshold);
    EXPECT_TRUE(gtfo::IsEqual(bound3Norm.GetSurfaceNormal(Eigen::Vector3d(0.0, 0.0, 1.0)), Eigen::Vector3d(0.0, 0.0, 1.0)));
    EXPECT_TRUE(gtfo::IsEqual(bound3Norm.GetSurfaceNormal(Eigen::Vector3d(1.0, 1.0, 1.0) / Eigen::Vector3d(1.0, 1.0, 1.0).lpNorm<3>()), Eigen::Vector3d(1.0, 1.0, 1.0).normalized()));
}