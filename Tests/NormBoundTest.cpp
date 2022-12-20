#include <gtest/gtest.h>

#include "../gtfo.hpp"

//----------------------------------------------------------------------------------------------------
// Contains Tests
//----------------------------------------------------------------------------------------------------
TEST(NormBoundTest, Contains1D) {
    const std::array<double, 5> thresholds = {0.1, 0.5, 1.0, 2.0, 10.0};
    const Eigen::Matrix<double, 1, 1> test_point_1d(-1.0);

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
    const Eigen::Matrix<double, 2, 1> test_point_2d(-1.0, 0.0);

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
    const Eigen::Matrix<double, 3, 1> test_point_3d(-1.0, 0.0, 0.0);

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

//----------------------------------------------------------------------------------------------------
// Nearest Point Tests
//----------------------------------------------------------------------------------------------------
TEST(NormBoundTest, NearestPointWithinBound1D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 1> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(-0.5))));

    // 3-Norm
    const gtfo::NormBound<3, 1> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
}

TEST(NormBoundTest, NearestPointWithinBound2D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 2> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(0.2, 0.2))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector2d(10.0, 10.0), Eigen::Vector2d(0.0, 0.0))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector2d(10.0, 0.1), Eigen::Vector2d(-0.8, -0.1))));

    // 3-Norm
    const gtfo::NormBound<3, 2> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(0.2, 0.2))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector2d(10.0, 10.0), Eigen::Vector2d(0.0, 0.0))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector2d(10.0, 0.1), Eigen::Vector2d(-0.8, -0.1))));
}

TEST(NormBoundTest, NearestPointWithinBound3D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 3> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector3d(10.0, 10.0, 10.0), Eigen::Vector3d(0.0, 0.0, 0.0))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector3d(5.0, -3.0, 2.0), Eigen::Vector3d(-0.3, 0.4, 0.1))));
    EXPECT_TRUE(bound2Norm.Contains(bound2Norm.GetNearestPointWithinBound(Eigen::Vector3d(0.0, 100.0, 0.0), Eigen::Vector3d(0.2, -0.3, 0.2))));
    
    // 3-Norm
    const gtfo::NormBound<3, 3> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector3d(10.0, 10.0, 10.0), Eigen::Vector3d(0.0, 0.0, 0.0))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector3d(5.0, -3.0, 2.0), Eigen::Vector3d(-0.3, 0.4, 0.1))));
    EXPECT_TRUE(bound3Norm.Contains(bound3Norm.GetNearestPointWithinBound(Eigen::Vector3d(0.0, 100.0, 0.0), Eigen::Vector3d(0.2, -0.3, 0.2))));
}

//----------------------------------------------------------------------------------------------------
// At Boundary Tests
//----------------------------------------------------------------------------------------------------
TEST(NormBoundTest, IsAtBoundary1D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 1> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(-1.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(0.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(2.0)));

    // 3-Norm
    const gtfo::NormBound<3, 1> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(-1.0)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(0.0)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Matrix<double, 1, 1>(2.0)));
}

TEST(NormBoundTest, IsAtBoundary2D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 2> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Vector2d(std::sqrt(2.0)/2.0, -std::sqrt(2.0)/2.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Vector2d(1.0, 0.1)));

    // 3-Norm
    const gtfo::NormBound<3, 2> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Vector2d(0.3, 0.9909)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Vector2d(0.3, 1.0)));
}

TEST(NormBoundTest, IsAtBoundary3D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 3> bound2Norm(threshold);
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Vector3d(0.0, -1.0, 0.0)));
    EXPECT_TRUE(bound2Norm.IsAtBoundary(Eigen::Vector3d(std::sqrt(3.0)/3.0, -std::sqrt(3.0)/3.0, std::sqrt(3.0)/3.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Vector3d(1.0, 1.0, 1.0)));
    EXPECT_FALSE(bound2Norm.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 0.9)));

    // 3-Norm
    const gtfo::NormBound<3, 3> bound3Norm(threshold);
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_TRUE(bound3Norm.IsAtBoundary(Eigen::Vector3d(0.6933, -0.6933, -0.6933)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Vector3d(0.6934, -0.6934, -0.6934)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Vector3d(1.0, 1.0, 1.0)));
    EXPECT_FALSE(bound3Norm.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 0.9)));
}

//----------------------------------------------------------------------------------------------------
// Surface Normals Tests
//----------------------------------------------------------------------------------------------------
TEST(NormBoundTest, GetSurfaceNormals1D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 1> bound2Norm(threshold);
    std::vector<Eigen::Matrix<double, 1, 1>> surface_normals = bound2Norm.GetSurfaceNormals(Eigen::Matrix<double, 1, 1>(1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Matrix<double, 1, 1>(1.0)));

    surface_normals = bound2Norm.GetSurfaceNormals(Eigen::Matrix<double, 1, 1>(-1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Matrix<double, 1, 1>(-1.0)));

    // 3-Norm
    const gtfo::NormBound<3, 1> bound3Norm(threshold);
    surface_normals = bound3Norm.GetSurfaceNormals(Eigen::Matrix<double, 1, 1>(1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Matrix<double, 1, 1>(1.0)));

    surface_normals = bound3Norm.GetSurfaceNormals(Eigen::Matrix<double, 1, 1>(-1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Matrix<double, 1, 1>(-1.0)));
}

TEST(NormBoundTest, GetSurfaceNormals2D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 2> bound2Norm(threshold);
    std::vector<Eigen::Vector2d> surface_normals = bound2Norm.GetSurfaceNormals(Eigen::Vector2d(0.0, 1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector2d(0.0, 1.0)));

    surface_normals = bound2Norm.GetSurfaceNormals(Eigen::Vector2d(1.0, 1.0).normalized());
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector2d(1.0, 1.0).normalized()));

    surface_normals = bound2Norm.GetSurfaceNormals(Eigen::Vector2d(-1.0, 0.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector2d(-1.0, 0.0)));
    
    // 3-Norm
    const gtfo::NormBound<3, 2> bound3Norm(threshold);
    surface_normals = bound3Norm.GetSurfaceNormals(Eigen::Vector2d(0.0, 1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector2d(0.0, 1.0)));

    surface_normals = bound3Norm.GetSurfaceNormals(Eigen::Vector2d(0.7937, 0.7937));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector2d(1.0, 1.0).normalized()));

    surface_normals = bound3Norm.GetSurfaceNormals(Eigen::Vector2d(-1.0, 0.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector2d(-1.0, 0.0)));
}

TEST(NormBoundTest, GetSurfaceNormals3D) {
    static constexpr double threshold = 1.0;

    // 2-Norm
    const gtfo::NormBound<2, 3> bound2Norm(threshold);
    std::vector<Eigen::Vector3d> surface_normals = bound2Norm.GetSurfaceNormals(Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector3d(0.0, 0.0, 1.0)));

    surface_normals = bound2Norm.GetSurfaceNormals(Eigen::Vector3d(1.0, 1.0, 1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector3d(1.0, 1.0, 1.0).normalized()));

    // 3-Norm
    const gtfo::NormBound<3, 3> bound3Norm(threshold);
    surface_normals = bound3Norm.GetSurfaceNormals(Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector3d(0.0, 0.0, 1.0)));

    surface_normals = bound3Norm.GetSurfaceNormals(Eigen::Vector3d(1.0, 1.0, 1.0) / Eigen::Vector3d(1.0, 1.0, 1.0).lpNorm<3>());
    EXPECT_TRUE(surface_normals.size() == 1 && gtfo::IsEqual(surface_normals[0], Eigen::Vector3d(1.0, 1.0, 1.0).normalized()));
}