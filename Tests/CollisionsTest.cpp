#include <gtest/gtest.h>
#include "../gtfo.hpp"

TEST(CollisionsTest, PointToPoint)
{
    gtfo::collision::Manipulator manipulator(std::vector<Eigen::Vector3d>{
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
    const std::vector<gtfo::collision::Segment<>> collisions = scene.GetCollisions(0);
    EXPECT_EQ(collisions.size(), 1);
    EXPECT_TRUE(gtfo::IsEqual(collisions[0].Start(), Eigen::Vector3d(1.0, 2.0, 3.0)));
    EXPECT_TRUE(gtfo::IsEqual(collisions[0].End(), Eigen::Vector3d::Zero()));
}

TEST(CollisionsTest, SegmentToPoint)
{
    gtfo::collision::Manipulator manipulator(std::vector<Eigen::Vector3d>{
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
    gtfo::collision::Segment<> segment = scene1.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.Start(), Eigen::Vector3d(0.0, 0.2, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.End(), Eigen::Vector3d(1.0, 0.2, 0.0)));

    // Segment obstacle beyond the endpoint of segment
    gtfo::collision::Obstacle obstacle2(std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(1.0, 10.0, 0.0)
    });
    gtfo::collision::Scene scene2(manipulator, obstacle2);
    scene2.SetCollisionDetectionThresholds(100.0, 100.0);
    scene2.ComputeCollisions();
    EXPECT_EQ(scene2.GetCollisions(0).size(), 1);
    segment = scene2.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.Start(), Eigen::Vector3d(0.0, 1.0, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.End(), Eigen::Vector3d(1.0, 10.0, 0.0)));

    // Move the manipulator to the obstacle
    scene2.UpdateVertices(0, std::vector<Eigen::Vector3d>{
        Eigen::Vector3d::Zero(), 
        Eigen::Vector3d(1.0, 10.0, 0.0)
    });
    scene2.ComputeCollisions();
    EXPECT_EQ(scene2.GetCollisions(0).size(), 1);
    segment = scene2.GetCollisions(0)[0];
    EXPECT_TRUE(gtfo::IsEqual(segment.Start(), Eigen::Vector3d(1.0, 10.0, 0.0)));
    EXPECT_TRUE(gtfo::IsEqual(segment.End(), Eigen::Vector3d(1.0, 10.0, 0.0)));
}