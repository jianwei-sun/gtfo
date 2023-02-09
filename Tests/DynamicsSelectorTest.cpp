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
    std::cout << selector.GetPosition().transpose() << "\n" << system_1.GetPosition().transpose() << "\n";
    EXPECT_TRUE(gtfo::IsEqual(selector.GetVelocity(), system_2.GetVelocity()));
    EXPECT_TRUE(gtfo::IsEqual(selector.GetAcceleration(), system_2.GetAcceleration()));
}