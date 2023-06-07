#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(CollisionDetectionTest, PointToPoint)
{
    gtfo::collision::Manipulator<6, 1> manipulator(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 2.0, 3.0)
    });
    gtfo::collision::Obstacle obstacle(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d::Zero()
    });
    
    gtfo::collision::Scene scene(manipulator);
    scene.AddEntity(obstacle);
    scene.SetCollisionDetectionThresholds(0.1, 0.1);

    scene.ComputeCollisions();
    EXPECT_TRUE(scene.GetCollisions(0).empty());

    scene.SetCollisionDetectionThresholds(10.0, 10.0);
    scene.ComputeCollisions();
    const auto collisions = scene.GetCollisions(0);
    EXPECT_EQ(collisions.size(), 1);
    EXPECT_TRUE(gtfo::IsEqual(collisions[0].location_, Eigen::Vector3d(1.0, 2.0, 3.0)));
    EXPECT_TRUE(gtfo::IsEqual(collisions[0].direction_, -Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(CollisionDetectionTest, SegmentToPoint)
{
    gtfo::collision::Manipulator<6, 1> manipulator(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d::Zero(), 
        Eigen::Vector3d(0.0, 1.0, 0.0)
    });

    // First obstacle projects onto the segment
    gtfo::collision::Obstacle obstacle1(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 0.2, 0.0)
    });
    gtfo::collision::Scene scene1(manipulator, obstacle1);
    scene1.SetCollisionDetectionThresholds(100.0, 100.0);
    scene1.ComputeCollisions();
    EXPECT_EQ(scene1.GetCollisions(0).size(), 1);
    auto segment = scene1.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Eigen::Vector3d(0.0, 0.2, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Eigen::Vector3d(1.0, 0.0, 0.0)));

    // Segment obstacle beyond the endpoint of segment
    gtfo::collision::Obstacle obstacle2(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 10.0, 0.0)
    });
    gtfo::collision::Scene scene2(manipulator, obstacle2);
    scene2.SetCollisionDetectionThresholds(100.0, 100.0);
    scene2.ComputeCollisions();
    EXPECT_EQ(scene2.GetCollisions(0).size(), 1);
    segment = scene2.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Eigen::Vector3d(0.0, 1.0, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Eigen::Vector3d(1.0, 9.0, 0.0)));

    // Move the manipulator to the obstacle
    scene2.UpdateVertices(0, std::vector<Eigen::Vector3d>{
        Eigen::Vector3d::Zero(), 
        Eigen::Vector3d(1.0, 10.0, 0.0)
    });
    scene2.ComputeCollisions();
    EXPECT_EQ(scene2.GetCollisions(0).size(), 1);
    segment = scene2.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Eigen::Vector3d(1.0, 10.0, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Eigen::Vector3d(0.0, 0.0, 0.0)));
}

TEST(CollisionDetectionTest, SegmentToSegment)
{
    gtfo::collision::Manipulator<6, 1> manipulator(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d::Zero(), 
        Eigen::Vector3d(0.0, 1.0, 0.0)
    });

    gtfo::collision::Obstacle obstacle1(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 0.5, -1.0),
        Eigen::Vector3d(-1.0, 0.5, -1.0)
    });
    gtfo::collision::Scene scene1(manipulator, obstacle1);
    scene1.SetCollisionDetectionThresholds(100.0, 100.0);
    scene1.ComputeCollisions();
    EXPECT_EQ(scene1.GetCollisions(0).size(), 1);
    auto segment = scene1.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Eigen::Vector3d(0.0, 0.5, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Eigen::Vector3d(0.0, 0.0, -1.0)));

    // Now move the manipulator over
    scene1.GetFreeEntity(0)->UpdateVertices({
        Eigen::Vector3d(2.0, 0.0, 0.0),
        Eigen::Vector3d(2.0, 1.0, 0.0)
    });
    scene1.ComputeCollisions();
    EXPECT_EQ(scene1.GetCollisions(0).size(), 1);
    segment = scene1.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Eigen::Vector3d(2.0, 0.5, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Eigen::Vector3d(-1.0, 0.0, -1.0)));

    // Now make the manipulator parallel 
    scene1.GetFreeEntity(0)->UpdateVertices({
        Eigen::Vector3d(0.0, 0.5, 0.0),
        Eigen::Vector3d(2.0, 0.5, 0.0)
    });
    scene1.ComputeCollisions();
    EXPECT_EQ(scene1.GetCollisions(0).size(), 1);
    segment = scene1.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Eigen::Vector3d(0.5, 0.5, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Eigen::Vector3d(0.0, 0.0, -1.0)));

    // Now make the manipulator intersect with the obstacle
    scene1.GetFreeEntity(0)->UpdateVertices({
        Eigen::Vector3d(1.0, 0.0, -1.0),
        Eigen::Vector3d(1.0, 2.0, -1.0)
    });
    scene1.ComputeCollisions();
    EXPECT_EQ(scene1.GetCollisions(0).size(), 1);
    segment = scene1.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.location_, Eigen::Vector3d(1.0, 0.5, -1.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.direction_, Eigen::Vector3d(0.0, 0.0, 0.0)));
}