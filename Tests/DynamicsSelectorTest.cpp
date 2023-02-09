#include <gtest/gtest.h>

#include "../gtfo.hpp"


TEST(DynamicsSelectorTest, SelectWorks)
{
    gtfo::PointMassFirstOrder<2> system_1((gtfo::FirstOrderParameters<double>()));
    gtfo::PointMassSecondOrder<2> system_2((gtfo::SecondOrderParameters<double>()));
    
    gtfo::DynamicsSelector<
        gtfo::PointMassFirstOrder<2>,
        gtfo::PointMassSecondOrder<2>>
    selector(system_1, system_2);

    const Eigen::Vector2d force(1.0, -0.5);

    EXPECT_TRUE(selector.Select(1));

    for(unsigned i = 0; i < 5; ++i){
        system_2.Step(force);
        selector.Step(force);
    }

    EXPECT_TRUE(gtfo::IsEqual(selector.GetPosition(), system_2.GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(selector.GetVelocity(), system_2.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(selector.GetAcceleration(), system_2.GetAcceleration()));

    EXPECT_TRUE(selector.Select(0));
    EXPECT_TRUE(gtfo::IsEqual(selector.GetPosition(), system_2.GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(selector.GetVelocity(), system_2.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(selector.GetAcceleration(), system_2.GetAcceleration()));
}

TEST(DynamicsSelectorTest, NestedSelectors)
{
    gtfo::PointMassSecondOrder<2> system_1((gtfo::SecondOrderParameters<double>()));
    gtfo::PointMassSecondOrder<2> system_2((gtfo::SecondOrderParameters<double>()));
    gtfo::DynamicsSelector<
        gtfo::PointMassSecondOrder<2>,
        gtfo::PointMassSecondOrder<2>>
    selector_1(system_1, system_2);

    gtfo::DynamicsSelector<
        gtfo::DynamicsSelector<
            gtfo::PointMassSecondOrder<2>,
            gtfo::PointMassSecondOrder<2>>,
        gtfo::PointMassSecondOrder<2>>
    selector_2(selector_1, system_2);

    const Eigen::Vector2d force(1.0, -0.5);

    for(unsigned i = 0; i < 5; ++i){
        system_2.Step(force);
        selector_2.Step(force);
    }

    EXPECT_TRUE(selector_2.Select(1));
    EXPECT_TRUE(gtfo::IsEqual(selector_2.GetPosition(), system_2.GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(selector_2.GetVelocity(), system_2.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(selector_2.GetAcceleration(), system_2.GetAcceleration()));
}

TEST(DynamicsSelectorTest, SelectToBounded)
{
    gtfo::PointMassSecondOrder<2> system_1((gtfo::SecondOrderParameters<double>()));
    gtfo::PointMassSecondOrder<2> system_2((gtfo::SecondOrderParameters<double>()));
    const gtfo::RectangleBound<2> hard_bound(Eigen::Vector2d(0.2, 0.1));
    system_2.SetHardBound(hard_bound);
    gtfo::DynamicsSelector<
        gtfo::PointMassSecondOrder<2>,
        gtfo::PointMassSecondOrder<2>>
    selector(system_1, system_2);

    // Out of bounds selection
    EXPECT_FALSE(selector.Select(2));

    const Eigen::Vector2d force(1.0, -0.5);

    for(unsigned i = 0; i < 5; ++i){
        selector.Step(force);
    }

    // Switch to the bounded system
    selector.Select(1);

    // Verify that the selector's state are within the hard bound
    EXPECT_TRUE(hard_bound.Contains(selector.GetPosition()));
}

TEST(DynamicsSelectorTest, DynamicsVectorCombination)
{
    gtfo::PointMassSecondOrder<1> system_1d((gtfo::SecondOrderParameters<double>()));
    gtfo::PointMassFirstOrder<2> system_2d((gtfo::FirstOrderParameters<double>()));
    gtfo::DynamicsVector<
        gtfo::PointMassSecondOrder<1>,
        gtfo::PointMassFirstOrder<2>> 
    vector_system_3d(system_1d, system_2d);

    gtfo::PointMassSecondOrder<3> system_3d((gtfo::SecondOrderParameters<double>()));

    gtfo::DynamicsSelector<
        gtfo::DynamicsVector<
            gtfo::PointMassSecondOrder<1>,
            gtfo::PointMassFirstOrder<2>>,
        gtfo::PointMassSecondOrder<3>>
    selector(vector_system_3d, system_3d);

    const Eigen::Vector3d force(1.5, -2.3, 0.75);

    selector.Select(1);
    for(unsigned i = 0; i < 5; ++i){
        selector.Step(force);
        system_1d.Step(force.head<1>());
    }

    // Out of bounds selection
    EXPECT_FALSE(selector.Select(2));

    selector.Select(0);
    EXPECT_TRUE(gtfo::IsEqual(selector.GetPosition().head<1>(), system_1d.GetPosition()));
    EXPECT_TRUE(gtfo::IsEqual(selector.GetVelocity().head<1>(), system_1d.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(selector.GetAcceleration().head<1>(), system_1d.GetAcceleration()));
}