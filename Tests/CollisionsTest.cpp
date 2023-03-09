#include <gtest/gtest.h>
#include "../gtfo.hpp"

TEST(CollisionsTest, PointToPoint)
{
    gtfo::collision::Manipulator manipulator(std::vector<Eigen::Vector3d>{Eigen::Vector3d(1.0, 2.0, 3.0)});
    gtfo::collision::Obstacle obstacle(std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()});
    
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