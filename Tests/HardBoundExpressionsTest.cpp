#include <gtest/gtest.h>
#include "../gtfo.hpp"

// Verifies that a union of hard bounds behaves correctly
TEST(HardBoundExpressionTest, BoundsSideBySideUnion){
    // Create two bounds that are tangent to each other in the x direction
    const gtfo::RectangleBound<2> bound1(Eigen::Vector2d::Ones());
    const gtfo::RectangleBound<2> bound2(Eigen::Vector2d::Ones(), Eigen::Vector2d::UnitX() * 2.0);
    const auto bound = bound1 | bound2;

    const Eigen::Vector2d corner_test_point(1.0, 1.0);
    const Eigen::Vector2d side_test_point(1.0, 0.0);

    EXPECT_TRUE(bound.Contains(corner_test_point));
    EXPECT_TRUE(bound.Contains(side_test_point));

    EXPECT_TRUE(bound.IsAtBoundary(corner_test_point));
    EXPECT_FALSE(bound.IsAtBoundary(side_test_point));

    const auto corner_surface_normals = bound.GetSurfaceNormals(corner_test_point);
    EXPECT_TRUE(corner_surface_normals.Contains(Eigen::Vector2d(0.0, 1.0)));

    const auto side_surface_normals = bound.GetSurfaceNormals(side_test_point);
    EXPECT_TRUE(side_surface_normals.IsEmpty());
}

// Verifies that a union of hard bounds in a different shape behaves correctly
TEST(HardBoundExpressionTest, BoundsOverlappingUnion){
    const gtfo::RectangleBound<2> bound1(Eigen::Vector2d(1.0, 0.5), Eigen::Vector2d(1.0, 0.5));
    const gtfo::RectangleBound<2> bound2(Eigen::Vector2d(0.5, 1.0), Eigen::Vector2d(1.5, 1.0));
    const auto bound = bound1 | bound2;

    // Check the concave corner point
    const Eigen::Vector2d concave_corner_test_point(1.0, 1.0);
    EXPECT_TRUE(bound.IsAtBoundary(concave_corner_test_point));

    const auto surface_normals_at_concave = bound.GetSurfaceNormals(concave_corner_test_point);
    EXPECT_TRUE(surface_normals_at_concave.Contains(Eigen::Vector2d(0.0, 1.0)));
    EXPECT_TRUE(surface_normals_at_concave.Contains(Eigen::Vector2d(-1.0, 0.0)));
    
    // Now verify the convex corner point
    const Eigen::Vector2d convex_corner_test_point(2.0, 0.0);
    EXPECT_TRUE(bound.IsAtBoundary(convex_corner_test_point));

    const auto surface_normals_at_convex = bound.GetSurfaceNormals(convex_corner_test_point);
    EXPECT_TRUE(surface_normals_at_convex.Contains(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_convex.Contains(Eigen::Vector2d(0.0, -1.0)));
}

// Verifies that an intersection of hard bounds behaves correctly
TEST(HardBoundExpressionTest, BoundsSideBySideIntersection){
    // Create two bounds that are tangent to each other in the x direction
    const gtfo::RectangleBound<2> bound1(Eigen::Vector2d::Ones());
    const gtfo::RectangleBound<2> bound2(Eigen::Vector2d::Ones(), Eigen::Vector2d::UnitX() * 2.0);

    // The resulting bound is a 1D line from (1,-1) to (1,1)
    const auto bound = bound1 & bound2;

    // Check the test point at (1,1)
    const Eigen::Vector2d top_test_point(1.0, 1.0);
    EXPECT_TRUE(bound.IsAtBoundary(top_test_point));
    const auto surface_normals_at_top = bound.GetSurfaceNormals(top_test_point);
    EXPECT_TRUE(surface_normals_at_top.Contains(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_top.Contains(Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_top.Contains(Eigen::Vector2d(0.0, 1.0)));

    // Check the test point at (1,0)
    const Eigen::Vector2d middle_test_point(1.0, 0.0);
    EXPECT_TRUE(bound.IsAtBoundary(middle_test_point));
    const auto surface_normals_at_middle = bound.GetSurfaceNormals(middle_test_point);
    EXPECT_TRUE(surface_normals_at_middle.Contains(Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_middle.Contains(Eigen::Vector2d(1.0, 0.0)));

    // Check the test point at (1,-1)
    const Eigen::Vector2d bottom_test_point(1.0, -1.0);
    EXPECT_TRUE(bound.IsAtBoundary(middle_test_point));
    const auto surface_normals_at_bottom = bound.GetSurfaceNormals(bottom_test_point);
    EXPECT_TRUE(surface_normals_at_bottom.Contains(Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_bottom.Contains(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_bottom.Contains(Eigen::Vector2d(0.0, -1.0)));
}

TEST(HardBoundExpressionTest, BoundsOverlappingIntersection){
    const gtfo::RectangleBound<2> bound1(Eigen::Vector2d(1.0, 0.5), Eigen::Vector2d(1.0, 0.5));
    const gtfo::RectangleBound<2> bound2(Eigen::Vector2d(0.5, 1.0), Eigen::Vector2d(1.5, 1.0));

    // The resulting bound is a unit square centered at (1.5, 0.5)
    const auto bound = bound1 & bound2;

    // Ensure points that are on the individual bounds are not on the combined bound
    EXPECT_TRUE(bound1.IsAtBoundary(Eigen::Vector2d::Zero()));
    EXPECT_FALSE(bound2.IsAtBoundary(Eigen::Vector2d::Zero()));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Vector2d::Zero()));

    EXPECT_FALSE(bound1.IsAtBoundary(Eigen::Vector2d(1.0, 2.0)));
    EXPECT_TRUE(bound2.IsAtBoundary(Eigen::Vector2d(1.0, 2.0)));
    EXPECT_FALSE(bound.IsAtBoundary(Eigen::Vector2d(1.0, 2.0)));

    EXPECT_TRUE(bound1.IsAtBoundary(Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(bound2.IsAtBoundary(Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(bound.IsAtBoundary(Eigen::Vector2d(1.0, 1.0)));

    // Check points along the unit square resulting from the intersection,
    // starting with the top left
    const auto surface_normals_at_top_left = bound.GetSurfaceNormals(Eigen::Vector2d(1.0, 1.0));
    EXPECT_TRUE(surface_normals_at_top_left.Contains(Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_top_left.Contains(Eigen::Vector2d(0.0, 1.0)));

    // Midpoint on left side
    const auto surface_normals_at_left = bound.GetSurfaceNormals(Eigen::Vector2d(1.0, 0.5));
    EXPECT_TRUE(surface_normals_at_left.Contains(Eigen::Vector2d(-1.0, 0.0)));

    // Bottom left corner
    const auto surface_normals_at_bottom_left = bound.GetSurfaceNormals(Eigen::Vector2d(1.0, 0.0));
    EXPECT_TRUE(surface_normals_at_bottom_left.Contains(Eigen::Vector2d(-1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_bottom_left.Contains(Eigen::Vector2d(0.0, -1.0)));

    // Midpoint on bottom side
    const auto surface_normals_at_bottom = bound.GetSurfaceNormals(Eigen::Vector2d(1.5, 0.0));
    EXPECT_TRUE(surface_normals_at_bottom.Contains(Eigen::Vector2d(0.0, -1.0)));

    // Bottom right corner
    const auto surface_normals_at_bottom_right = bound.GetSurfaceNormals(Eigen::Vector2d(2.0, 0.0));
    EXPECT_TRUE(surface_normals_at_bottom_right.Contains(Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(surface_normals_at_bottom_right.Contains(Eigen::Vector2d(0.0, -1.0)));
}

// Move a virtual mass through an L-shaped region formed by a union of two bounds
TEST(HardBoundExpressionTest, SecondOrderSystem2DLShapedBound){
    const gtfo::RectangleBound<2> bound1(Eigen::Vector2d(2.0, 0.5), Eigen::Vector2d(2.0, 0.5));
    const gtfo::RectangleBound<2> bound2(Eigen::Vector2d(0.5, 2.0), Eigen::Vector2d(3.5, 2.0));

    // Test with a more complex bound that is geometrically equivalent to bound1 | bound2
    // const auto bound = bound1 & bound1 | bound2 & bound2 | bound1;
    const auto bound = bound1 | bound2;

    std::cout << "Creating system\n";
    gtfo::PointMassSecondOrder<2> system;
    system.SetParameters(gtfo::SecondOrderParameters<double>());
    std::cout << "Setting system hard bound\n";
    system.SetHardBound(bound);

    const Eigen::Vector2d force(1.0, 1.0);

    for (size_t i = 0; i < 15; ++i)
    {
        std::cout << "Position: " << system.GetPosition().transpose() << "\n";
        //std::cout << "Velocity: " << system.GetVelocity().transpose() << "\n";
        system.Step(force);
    }

    std::cout << "Final position: " << system.GetPosition().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d(4.0, 4.0)));

    std::cout << "Final velocity: " << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));
    
    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}

// Move a virtual mass through a small opening between bounds
TEST(HardBoundExpressionTest, SecondOrderSystem2DSmallOpening){
    const gtfo::NormBound<2,2> bound1(1.0);
    const gtfo::NormBound<2,2> bound2(1.0, Eigen::Vector2d(2.0, 0.0));

    const auto bound = bound1 | bound2;

    gtfo::PointMassSecondOrder<2> system;
    system.SetParameters(gtfo::SecondOrderParameters<double>());
    system.SetHardBound(bound);

    const Eigen::Vector2d force(1.0, 0.0);

    for (size_t i = 0; i < 10; ++i)
    {
        system.Step(force);
    }

    std::cout << "Final position: " << system.GetPosition().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d(3.0, 0.0)));

    std::cout << "Final velocity: " << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));
    
    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}

// Prevent going out of bounds at corners between bounds
TEST(HardBoundExpressionTest, EscapingAtCorners){
    const gtfo::RectangleBound<2> bound1(Eigen::Vector2d::Ones(), -Eigen::Vector2d::Ones());
    const gtfo::RectangleBound<2> bound2(Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones());

    const auto bound = bound1 | bound2;

    gtfo::PointMassSecondOrder<2> system;
    system.SetParameters(gtfo::SecondOrderParameters<double>());
    system.SetHardBound(bound);

    const Eigen::Vector2d force(-1.0, 1.0);

    // Test a range of forces that all point out of the bound expression
    for (double angle = 95; angle < 175; angle += 5.0)
    {
        const double angle_radian = angle * std::atan(1.0) / 45.0;
        const Eigen::Vector2d force = Eigen::Vector2d(cos(angle_radian), sin(angle_radian));
        system.Step(force);
    }

    std::cout << "Final position: " << system.GetPosition().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector2d::Zero()));

    std::cout << "Final velocity: " << system.GetVelocity().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), Eigen::Vector2d::Zero()));
    
    std::cout << "Final acceleration: " << system.GetAcceleration().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), Eigen::Vector2d::Zero()));
}