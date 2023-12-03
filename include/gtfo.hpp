//----------------------------------------------------------------------------------------------------
// File: gtfo.hpp
// Desc: Main header file for including the gtfo library 
//----------------------------------------------------------------------------------------------------
#pragma once

#include "../Core/Utils/ClosestVector.hpp"
#include "../Core/Utils/Constants.hpp"
#include "../Core/Utils/Comparisons.hpp"
#include "../Core/Utils/Functions.hpp"
#include "../Core/Utils/Containers.hpp"

#include "../Core/Models/PointMassFirstOrder.hpp"
#include "../Core/Models/PointMassSecondOrder.hpp"
#include "../Core/Models/RotationSecondOrder.hpp"
#include "../Core/Models/RigidBodySecondOrder.hpp"
#include "../Core/Models/HomingModel.hpp"
#include "../Core/Models/ConstantVelocityModel.hpp"
#include "../Core/Models/Mujoco/MujocoModel.hpp"

#include "../Core/Bounds/NormBound.hpp"
#include "../Core/Bounds/RectangleBound.hpp"

#include "../Core/Containers/DynamicsVector.hpp"
#include "../Core/Containers/DynamicsSelector.hpp"

#include "../Core/Filters/DiscreteTimeFilter.hpp"
#include "../Core/Filters/ButterworthLowPassFilter.hpp"
#include "../Core/Filters/RateLimiter.hpp"

#include "../Collision/Segment.hpp"
#include "../Collision/Manipulator.hpp"
#include "../Collision/Obstacle.hpp"
#include "../Collision/Scene.hpp"

#include "../Constraints/ManifoldConstraints.hpp"
