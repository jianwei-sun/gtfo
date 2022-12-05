#include <gtest/gtest.h>

#include "../gtfo.hpp"

TEST(NormBoundTest, Contains) {
    std::array<double, 5> thresholds = {0.1, 0.5, 1.0, 2.0, 10.0};

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