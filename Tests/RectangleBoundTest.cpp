#include <gtest/gtest.h>

#include "../gtfo.hpp"

//----------------------------------------------------------------------------------------------------
// Contains Tests
//----------------------------------------------------------------------------------------------------
TEST(RectangleBoundTest, Contains1D){
    const std::vector<double> thresholds = {0.1, 0.5, 1.0, 2.0, 10.0};
    const Eigen::Matrix<double, 1, 1> test_point_1d(-1.0);

    EXPECT_FALSE(gtfo::RectangleBound<1>(Eigen::Matrix<double, 1, 1>(thresholds[0])).Contains(test_point_1d));
    EXPECT_FALSE(gtfo::RectangleBound<1>(Eigen::Matrix<double, 1, 1>(thresholds[1])).Contains(test_point_1d));
    EXPECT_TRUE(gtfo::RectangleBound<1>(Eigen::Matrix<double, 1, 1>(thresholds[2])).Contains(test_point_1d));
    EXPECT_TRUE(gtfo::RectangleBound<1>(Eigen::Matrix<double, 1, 1>(thresholds[3])).Contains(test_point_1d));
    EXPECT_TRUE(gtfo::RectangleBound<1>(Eigen::Matrix<double, 1, 1>(thresholds[4])).Contains(test_point_1d));
}

TEST(RectangleBoundTest, Contains2D){
    const std::array<double, 5> thresholds = {0.1, 0.5, 1.0, 2.0, 10.0};
    const Eigen::Matrix<double, 2, 1> test_point_2d(-1.0, 0.0);

    EXPECT_FALSE(gtfo::RectangleBound<2>(Eigen::Vector2d::Ones() * thresholds[0]).Contains(test_point_2d));
    EXPECT_FALSE(gtfo::RectangleBound<2>(Eigen::Vector2d::Ones() * thresholds[1]).Contains(test_point_2d));
    EXPECT_TRUE(gtfo::RectangleBound<2>(Eigen::Vector2d::Ones() * thresholds[2]).Contains(test_point_2d));
    EXPECT_TRUE(gtfo::RectangleBound<2>(Eigen::Vector2d::Ones() * thresholds[3]).Contains(test_point_2d));
    EXPECT_TRUE(gtfo::RectangleBound<2>(Eigen::Vector2d::Ones() * thresholds[4]).Contains(test_point_2d));
}

TEST(RectangleBoundTest, Contains3D){
    const std::array<double, 5> thresholds = {0.1, 0.5, 1.0, 2.0, 10.0};
    const Eigen::Matrix<double, 3, 1> test_point_3d(-1.0, 0.0, 0.0);

    EXPECT_FALSE(gtfo::RectangleBound<3>(Eigen::Vector3d::Ones() * thresholds[0]).Contains(test_point_3d));
    EXPECT_FALSE(gtfo::RectangleBound<3>(Eigen::Vector3d::Ones() * thresholds[1]).Contains(test_point_3d));
    EXPECT_TRUE(gtfo::RectangleBound<3>(Eigen::Vector3d::Ones() * thresholds[2]).Contains(test_point_3d));
    EXPECT_TRUE(gtfo::RectangleBound<3>(Eigen::Vector3d::Ones() * thresholds[3]).Contains(test_point_3d));
    EXPECT_TRUE(gtfo::RectangleBound<3>(Eigen::Vector3d::Ones() * thresholds[4]).Contains(test_point_3d));
}

//----------------------------------------------------------------------------------------------------
// Nearest Point Tests
//----------------------------------------------------------------------------------------------------
TEST(RectangleBoundTest, NearestPointWithinBound1D){
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<1> bound(Eigen::Matrix<double, 1, 1>::Ones() * threshold);

    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(0.5))));
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Matrix<double, 1, 1>(-2.0), Eigen::Matrix<double, 1, 1>(-0.5))));
}   

TEST(RectangleBoundTest, NearestPointWithinBound2D){
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<2> bound(Eigen::Vector2d::Ones() * threshold);
    
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(0.4, 0.3))));
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Vector2d(2.0, 0.2), Eigen::Vector2d(-0.4, 0.3))));
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Vector2d(-2.0, -2.0), Eigen::Vector2d(-1.0, -1.0))));
}   

TEST(RectangleBoundTest, NearestPointWithinBound3D){
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<3> bound(Eigen::Vector3d::Ones() * threshold);
    
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Vector3d(0.4, 0.3, 0.2))));
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Vector3d(-12.0, 12.0, -5.0), Eigen::Vector3d(0.3, -0.2, 0.5))));
    EXPECT_TRUE(bound.Contains(bound.GetNearestPointWithinBound(Eigen::Vector3d(20.0, 20.0, 20.0), Eigen::Vector3d(-1.0, -1.0, -1.0))));
}   

//----------------------------------------------------------------------------------------------------
// At Boundary Tests
//----------------------------------------------------------------------------------------------------
TEST(RectangleBoundTest, IsAtBoundary1D) {
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<1> bound(Eigen::Matrix<double, 1, 1>::Ones() * threshold);

    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Matrix<double, 1, 1>(1.0)));
    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Matrix<double, 1, 1>(-1.0)));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Matrix<double, 1, 1>(0.0)));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Matrix<double, 1, 1>(2.0)));
}

TEST(RectangleBoundTest, IsAtBoundary2D) {
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<2> bound(Eigen::Vector2d::Ones() * threshold);

    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Vector2d(-1.0, 1.0)));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Vector2d(2.0, 1.0)));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Vector2d(0.0, -1.2)));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Vector2d(-1.0, 1.2)));
}

TEST(RectangleBoundTest, IsAtBoundary3D) {
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<3> bound(Eigen::Vector3d::Ones() * threshold);

    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Vector3d(1.0, 0.0, -1.0)));
    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Vector3d(-1.0, -1.0, -1.0)));
    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 1.0)));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Vector3d(0.0, 0.0, 0.9)));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Vector3d(1.0, 1.0, 1.1)));
}

//----------------------------------------------------------------------------------------------------
// Surface Normals Tests
//----------------------------------------------------------------------------------------------------
TEST(RectangleBoundTest, GetSurfaceNormals1D) {
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<1> bound(Eigen::Matrix<double, 1, 1>::Ones() * threshold);

    auto surface_normals = bound.GetSurfaceNormals(Eigen::Matrix<double, 1, 1>(1.0));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Matrix<double, 1, 1>(1.0)));

    surface_normals = bound.GetSurfaceNormals(Eigen::Matrix<double, 1, 1>(-1.0));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Matrix<double, 1, 1>(-1.0)));
}

TEST(RectangleBoundTest, GetSurfaceNormals2D) {
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<2> bound(Eigen::Vector2d::Ones() * threshold);

    auto surface_normals = bound.GetSurfaceNormals(Eigen::Vector2d(0.0, 1.0));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector2d(0.0, 1.0)));

    surface_normals = bound.GetSurfaceNormals(Eigen::Vector2d(-1.0, 0.0));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector2d(-1.0, 0.0)));

    surface_normals = bound.GetSurfaceNormals(Eigen::Vector2d(1.0, 0.99));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector2d(1.0, 0.0)));

    surface_normals = bound.GetSurfaceNormals(Eigen::Vector2d(1.0, -1.0));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector2d(0.0, -1.0)));
}

TEST(RectangleBoundTest, GetSurfaceNormals3D) {
    static constexpr double threshold = 1.0;
    const gtfo::RectangleBound<3> bound(Eigen::Vector3d::Ones() * threshold);

    auto surface_normals = bound.GetSurfaceNormals(Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector3d(0.0, 0.0, 1.0)));

    surface_normals = bound.GetSurfaceNormals(Eigen::Vector3d(-1.0, 0.5, -1.0));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector3d(-1.0, 0.0, 0.0)));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector3d(0.0, 0.0, -1.0)));

    surface_normals = bound.GetSurfaceNormals(Eigen::Vector3d(1.0, -1.0, 1.0));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector3d(0.0, -1.0, 0.0)));
    EXPECT_TRUE(surface_normals.Contains(Eigen::Vector3d(0.0, 0.0, 1.0)));
}