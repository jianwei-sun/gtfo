#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(CollisionDetectionTest, PointToPoint)
{
    using Vector3 = Eigen::Matrix<c_float, 3, 1>;

    gtfo::collision::Manipulator<6, 1, 6, c_float> manipulator(std::vector<Vector3>{
        Vector3(1.0, 2.0, 3.0)
    });
    gtfo::collision::Obstacle<c_float> obstacle(std::vector<Vector3>{
        Vector3::Zero()
    });
    
    gtfo::collision::Scene<c_float> scene(manipulator);
    scene.AddEntity(obstacle);
    scene.SetCollisionDetectionThresholds(0.1, 0.1);

    scene.ComputeCollisions();
    EXPECT_TRUE(scene.GetCollisions(0).empty());

    scene.SetCollisionDetectionThresholds(10.0, 10.0);
    scene.ComputeCollisions();
    const auto collisions = scene.GetCollisions(0);
    EXPECT_EQ(collisions.size(), 1);
    EXPECT_TRUE(gtfo::IsEqual(collisions[0].location_, Vector3(1.0, 2.0, 3.0)));
    EXPECT_TRUE(gtfo::IsEqual(collisions[0].direction_, -Vector3(1.0, 2.0, 3.0)));
}

TEST(CollisionDetectionTest, SegmentToPoint)
{
    using Vector3 = Eigen::Matrix<c_float, 3, 1>;

    gtfo::collision::Manipulator<6, 1, 6, c_float> manipulator(std::vector<Vector3>{
        Vector3::Zero(), 
        Vector3(0.0, 1.0, 0.0)
    });

    // First obstacle projects onto the segment
    gtfo::collision::Obstacle<c_float> obstacle1(std::vector<Vector3>{
        Vector3(1.0, 0.2, 0.0)
    });
    gtfo::collision::Scene<c_float> scene1(manipulator, obstacle1);
    scene1.SetCollisionDetectionThresholds(100.0, 100.0);
    scene1.ComputeCollisions();
    EXPECT_EQ(scene1.GetCollisions(0).size(), 1);
    auto segment = scene1.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Vector3(0.0, 0.2, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Vector3(1.0, 0.0, 0.0)));

    // Segment obstacle beyond the endpoint of segment
    gtfo::collision::Obstacle<c_float> obstacle2(std::vector<Vector3>{
        Vector3(1.0, 10.0, 0.0)
    });
    gtfo::collision::Scene<c_float> scene2(manipulator, obstacle2);
    scene2.SetCollisionDetectionThresholds(100.0, 100.0);
    scene2.ComputeCollisions();
    EXPECT_EQ(scene2.GetCollisions(0).size(), 1);
    segment = scene2.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Vector3(0.0, 1.0, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Vector3(1.0, 9.0, 0.0)));

    // Move the manipulator to the obstacle
    scene2.UpdateVertices(0, std::vector<Vector3>{
        Vector3::Zero(), 
        Vector3(1.0, 10.0, 0.0)
    });
    scene2.ComputeCollisions();
    EXPECT_EQ(scene2.GetCollisions(0).size(), 1);
    segment = scene2.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Vector3(1.0, 10.0, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Vector3(0.0, 0.0, 0.0)));
}

TEST(CollisionDetectionTest, SegmentToSegment)
{
    using Vector3 = Eigen::Matrix<c_float, 3, 1>;

    gtfo::collision::Manipulator<6, 1, 6, c_float> manipulator(std::vector<Vector3>{
        Vector3::Zero(), 
        Vector3(0.0, 1.0, 0.0)
    });

    gtfo::collision::Obstacle<c_float> obstacle1(std::vector<Vector3>{
        Vector3(1.0, 0.5, -1.0),
        Vector3(-1.0, 0.5, -1.0)
    });
    gtfo::collision::Scene<c_float> scene(manipulator, obstacle1);
    scene.SetCollisionDetectionThresholds(100.0, 100.0);
    scene.ComputeCollisions();
    EXPECT_EQ(scene.GetCollisions(0).size(), 1);
    auto segment = scene.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Vector3(0.0, 0.5, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Vector3(0.0, 0.0, -1.0)));

    // Now move the manipulator over
    scene.GetFreeEntity(0)->UpdateVertices({
        Vector3(2.0, 0.0, 0.0),
        Vector3(2.0, 1.0, 0.0)
    });
    scene.ComputeCollisions();
    EXPECT_EQ(scene.GetCollisions(0).size(), 1);
    segment = scene.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Vector3(2.0, 0.5, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Vector3(-1.0, 0.0, -1.0)));

    // Now make the manipulator parallel 
    scene.GetFreeEntity(0)->UpdateVertices({
        Vector3(0.0, 0.5, 0.0),
        Vector3(2.0, 0.5, 0.0)
    });
    scene.ComputeCollisions();
    EXPECT_EQ(scene.GetCollisions(0).size(), 1);
    segment = scene.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Vector3(0.5, 0.5, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Vector3(0.0, 0.0, -1.0)));

    // Now make the manipulator intersect with the obstacle
    scene.GetFreeEntity(0)->UpdateVertices({
        Vector3(1.0, 0.0, -1.0),
        Vector3(1.0, 2.0, -1.0)
    });
    scene.ComputeCollisions();
    EXPECT_EQ(scene.GetCollisions(0).size(), 1);
    segment = scene.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Vector3(1.0, 0.5, -1.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Vector3(0.0, 0.0, 0.0)));
}