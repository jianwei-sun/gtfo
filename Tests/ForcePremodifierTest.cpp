#include <gtest/gtest.h>
#include "gtfo.hpp"

TEST(ForcePremodifierTest, FlipForces)
{
    gtfo::PointMassSecondOrder<2> system((gtfo::SecondOrderParameters<double>()));

    auto lambda = [](const Eigen::Vector2d& force, const gtfo::DynamicsBase<2>& system){
        return -force;
    };
    system.SetForcePremodifier(lambda);

    for(unsigned i = 0; i < 100; ++i){
        system.Step(Eigen::Vector2d::Ones());
    }
    EXPECT_TRUE((system.GetPosition().array() < 0.0).all());
}


TEST(ForcePremodifierTest, Containers)
{
    gtfo::DynamicsSelector<
        gtfo::DynamicsVector<
            gtfo::PointMassSecondOrder<2>,
            gtfo::PointMassFirstOrder<2>>,
        gtfo::PointMassSecondOrder<4>>
    system(
        gtfo::DynamicsVector<gtfo::PointMassSecondOrder<2>,gtfo::PointMassFirstOrder<2>>(
            (gtfo::SecondOrderParameters<double>()), (gtfo::FirstOrderParameters<double>())), 
        (gtfo::SecondOrderParameters<double>()));

    // Make the first system just pass in the opposite of the force in the first two coordinates
    system.GetModel<0>().SetForcePremodifier([](const Eigen::Vector4d& force, const gtfo::DynamicsBase<4>& system){
       return force.cwiseProduct(Eigen::Vector4d(-1.0, -1.0, 1.0, 1.0)).eval();
    });

    // Make the second system be a PD controller
    system.GetModel<1>().SetForcePremodifier([](const Eigen::Vector4d& force, const gtfo::DynamicsBase<4>& system){
       return -1.0 * system.GetPosition() - 0.5 * system.GetVelocity();
    });

    // Step the first system
    for(unsigned i = 0; i < 10; ++i){
        system.Step(Eigen::Vector4d::Ones());
    }

    // Ensure that the first second order model behaves correctly
    std::cout << system.GetPosition().block<2,1>(0,0) << std::endl;
    EXPECT_TRUE((system.GetPosition().block<2,1>(0,0).array() < 0.0).all());
    EXPECT_TRUE((system.GetPosition().block<2,1>(2,0).array() > 0.0).all());

    // Note that when switching to the second system, its initial conditions are that of the first system
    const Eigen::Vector4d first_position = system.GetPosition();
    const Eigen::Vector4d first_velocity = system.GetVelocity();
    const Eigen::Vector4d first_acceleration = system.GetAcceleration();
    EXPECT_TRUE(system.Select(1));
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), first_position));
    EXPECT_TRUE(gtfo::IsEqual(system.GetVelocity(), first_velocity));
    EXPECT_TRUE(gtfo::IsEqual(system.GetAcceleration(), first_acceleration));

    // Now step the second system
    for(unsigned i = 0; i < 20; ++i){
        system.Step(Eigen::Vector4d::Ones());
    }

    // Verify that the second system has stabilized to the origin
    EXPECT_TRUE(gtfo::IsEqual(system.GetPosition(), Eigen::Vector4d::Zero()));
}