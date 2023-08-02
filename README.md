# Generated Trajectories for Orthoses (GTFO)

`gtfo` is a safety-focused virtual dynamics library for physical human-robot interaction (pHRI) applications, written in C++. The library can generate virtual point-mass dynamics, impose soft and hard bounding regions, and perform multi-arm collision detection and avoidance. It can be used as a stand-alone reference generator for admittance control, or used as a safety layer to support custom applications.

Written by the [Bionics Lab](http://bionics.seas.ucla.edu/index.html) at UCLA.

## Installation

### Dependencies
This project requires `Eigen` version `3.4.0`, `MuJoCo` version `2.3.1`, and `OSQP` version `0.6.2` as dependencies. Ensure that they are already installed on your computer. If not, their source code can be made available in the `External` subdirectory as git submodules after updating:
```
git submodule update --init --recursive
```
Then, follow their instructions for building and installing.

### Getting the Library
Clone the repository to your project with:
```
git clone https://github.com/jianwei-sun/gtfo.git
```
Then, either include the project with `CMake`, or make sure the `gtfo/include` directory is available to your compiler (e.g., `-I gtfo/include` for `gcc`).
### Including in Your Project
`gtfo` is designed as a header-only C++ library, which means just including the header is sufficient: 
```c++
#include "gtfo.hpp"
```
If your project uses `CMake`, you can also include the library with:
```cmake
add_subdirectory(gtfo)
````
Then, it can be made available to targets with:
```cmake
target_link_libraries(${PROJECT_NAME} PRIVATE
    gtfo
)
```
Unit tests can be disabled by:
```cmake
set(GTFO_BUILD_TESTS OFF)
```
Note that in order to build the unit tests, `OSQP` must be configured to use `double` as the floating-point type, which is the default option (`-DFLOAT=OFF`).

# Usage
This section describes some common use cases for features of the library. More detailed use cases can be found in the unit tests, under `Tests`.
## Virtual Dynamics
In the most straight-forward case, the virtual dynamics can be stepped and its state queried:
```c++
// Create parameters for a second order model, which include: sampling period, mass, and damping
const gtfo::SecondOrderParameters<double> test_parameters(0.01, 10, 5);

// Create a virtual system in 3D
gtfo::PointMassSecondOrder<3, double> system(test_parameters);

// In a main control loop, step the dynamics and query its state
while(true){
    ...
    // force_input is assumed to have already been received 
    system.Step(force_input);

    // Now pass the reference state to your robot's controller
    position = system.GetPosition();
    velocity = system.GetVelocity();
    acceleration = system.GetAcceleration();
    ...
}
```
This example shows typical usage for a system with second-order dynamics. The template arguments to `gtfo::PointMassSecondOrder` configures the type of the `force_input` to be `Eigen::Matrix<double, 3, 1>`. The `position` and `velocity` are of the same type.

### Soft Start
Often when a pHRI experiment starts, we may want the virtual parameters (e.g. damping) to be much larger than usual and gradually decay to the nominal values, so that the sudden motion of the robot is not a surprise. This feature is available for all subclasses of `gtfo::PointMassBase`. The example below shows an example configuration:
```c++
const gtfo::SecondOrderParameters<float> nominal_parameters(0.001, 5, 5);
const gtfo::SecondOrderParameters<float> starting_parameters(0.001, 15, 15);

gtfo::PointMassSecondOrder<2, float> system(nominal_parameters);

// Configure soft-start for a duration of 3 seconds
system.ConfigureSoftStart(starting_parameters, 3.0);
```
Soft start linearly interpolates between the starting and nominal parameters over a duration specified by the user. The starting parameters and soft start duration are the arguments to the `ConfigureSoftStart` function.

### Vector Dynamics
In many cases we may want the different coordinates of a system to have different dynamics, e.g., an exoskeleton controlled in joint-space might want different stiffness for each of its joints. This can be achieved with the `gtfo::DynamicsVector` object:
```c++
// Construct two systems with default parameters
gtfo::PointMassSecondOrder<2> subsystem_2d(gtfo::SecondOrderParameters<double>());
gtfo::PointMassFirstOrder<1> subsystem_1d(gtfo::FirstOrderParameters<double>());

// Construct a 3D system in which the first two coordinates have the same dynamics, and the last one is different. subsystem_2d and subsystem_1d are copy-constructed into system, so the original instances are no longer needed
gtfo::DynamicsVector<
    gtfo::PointMassSecondOrder<2>,
    gtfo::PointMassFirstOrder<1>>
system(subsystem_2d, subsystem_1d);

// Use the vector system as a stand-alone system
while(true){
    system.Step(...);

    // The dimension of the state is the sum of all the dimensions of the subsystems
    const Eigen::Vector3d position = system.GetPosition();
    ...
}
```
A reference to any subsystem within the vector can be retrieved with `gtfo::DynamicsVector::GetModel<i>()`, where `i` is the index of the subsystem. 
### Dynamics Switching
It is often desired to switch between different reference signals in a pHRI scenario. For instance, switching between a nominal virtual dynamics reference and a homing reference to reset the robotic manipulators. This is the motivation behind the `gtfo::DynamicsSelector`:
```c++
// Construct the two types of references
gtfo::PointMassSecondOrder<3> nominal(gtfo::SecondOrderParameters<double>());
gtfo::HomingModel<3> homing(1.0, 1.0);

// Add them to a DynamicsSelector
gtfo::DynamicsSelector<
    gtfo::PointMassSecondOrder<3>,
    gtfo::HomingModel<3>>
system(nominal, homing);

// Nominal usage
while(nominal_usage){
    system.Step(...);
}

// Switch to homing and continue
system.Select(1);
while(going_home){
    system.Step(...);
}
```
Switching between systems automatically updates the new system's state to the old one's at the time of the switch, so discontinuities are avoided. Since `DynamicsVector` and `DynamicsSelector` are both considered dynamics models, they can be nested within each other to any arbitrary degree. 
## Custom Dynamics
In addition to the provided models:
- `PointMassSecondOrder`: second-order mass-damper system
- `PointMassFirstOrder`: first-order time-constant and dc-gain system
- `HomingModel`: creates continuous homing trajectories with constant acceleration in first and last 10%
- `ConstantVelocityModel`: constant velocity motions, useful for simple gui-commanded motions
- `MujocoModel`: a MuJoCo-compatible model, which uses MuJoCo to propagate dynamics

custom models for your application can also be created. All dynamics models inherit from `gtfo::DynamicsBase`, so custom models should extend the class and override the `Step` function. 

## Settings Bounds
`gtfo` allows for both soft and hard convex bounds to constrain the dynamics. The bound behavior (soft vs. hard) is independent of the bound's geometry. The `Core/Bounds` subdirectory contains two commonly used bound shapes: `NormBound` and `RectangleBound`.

### Bound Geometry
Both `NormBound` and `RectangleBound` are subclasses of `BoundBase`, and can be constructed by specifying their size:
```c++
// Construct a 2D norm bound (circle) with desired radius
const gtfo::NormBound<2> norm_bound(radius);

// Construct a 2D rectangle bound with bilateral limits length_1 and length_2
const gtfo::RectangleBound<2> rectangle_bound(Eigen::Vector2d(length_1, length_2));

```

### Associating Bounds with Dynamics
Once bound objects have been created, they can be set as soft and/or hard bounds for a particular dynamical system:
```c++
// Construct a 2D second-order dynamical system
gtfo::PointMassSecondOrder<2> system(gtfo::SecondOrderParameters<double>());

// Set a hard bound
system.SetHardBound(rectangle_bound);

// Set a soft bound with specified spring stiffness and damping
system.SetSoftBound(norm_bound, spring_constant, damping_constant)
```
A system does not require any bounds in order to operate. Typically, when both bound types are set, the soft bound is contained within the hard bound, otherwise the soft bound would never be reached.

If more complex bound behaviors are desired, such as different types of bounds in different coordinates or the ability to dynamically switch between bounds, `DynamicsVector` and/or `DynamicsSelector` should be used:
```c++
// Construct a system from two subsystems. Note that they are copied into system, so any bounds already associated with them are also copied
gtfo::DynamicsSelector<
    gtfo::PointMassSecondOrder<3>,
    gtfo::DynamicsVector<
        gtfo::PointMassSecondOrder<2>,
        gtfo::PointMassFirstOrder<1>>>
system(system_1, system_2);

// Bounds can still be set for them after they've been copied into system
system.GetModel<0>().SetHardBounds(bound_1);

// Any combination of bounding behavior can be achieved
system.GetModel<1>().SetHardBounds(bound_2);
system.GetModel<1>().SetSoftBounds(bound_3);
```

## Multi-Arm Collision Avoidance
Collision avoidance between multiple robotic arms, or even between a single arm and a human operator, should always be considered for safety. To facilitate this goal, the library implements collision avoidance as a standalone compatible feature. 

Collision avoidance works in two parts: detecting potential collisions, and then restricting velocities to prevent potential collision distances from becoming lower. Programmatically, this requires telling the library which entities exist in the scene and their pose. To do this, collision points are defined as key locations on the manipulator such that segments drawn between adjacent pairs of points roughly correspond to the geometry of the robot. Collision avoidance works by computing the minimum distance vector between these segments, and then restricting the manipulator's movement at the point of that vector in the direction of the vector.

### Configuration
The example below shows how to configure a two robot scene, each of which has two segments (3 collision points):
```c++
// Create two manipulator objects, each of which is 5 DOF and has 3 collision points
gtfo::collision::Manipulator<5, 2> left_arm(std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero()));
gtfo::collision::Manipulator<5, 2> right_arm(std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero()));
```
Note in the example above that the second template argument of 2 refers to the maximum expected collisions per segment. Since each arm has two segments, and self-collisions between adjacent segments belonging to the same arm are pruned, each segment can only collide with potentially two other segments. 

If either manipulator also has virtual dynamics, it is necessary to associate the dynamic model with the manipulator. Since collision avoidance affects the reference velocity, the state of the reference has to be updated accordingly. This can be done automatically by setting two lambdas:
```c++
// First argument is a pointer to the DynamicsBase object
left_arm.SetVirtualDynamics(&left_system, 
    // Second argument is a mapping function between the velocity in the virtual 
    // dynamics space to the velocity in joint space (where collision avoidance 
    // happens)
    [](const JointVector& virtual_velocity){
        return virtual_velocity;
    }, 
    // Third argument is the inverse mapping; from the velocity in joint space to 
    // the velocity of the dynamics
    [](const JointVector& joint_velocity){
        return joint_velocity;
    });
```
In the example above, the virtual dynamics are implemented in joint space, so the velocity mappings are just the identity maps. This step is required if collision avoidance is desired, but can be omitted if you only care about collision detection.

The last stage, which is required, is to add the manipulator entities into a virtual scene:
```c++
// Create a scene and add any entities, which are copied over
gtfo::collision::Scene<double> scene;
scene.AddEntity(left_arm);
scene.AddEntity(right_arm);

// Additional static entities can also be added
scene.AddEntity(obstacle_1);

// Lastly, update the collision detection thresholds
scene.SetCollisionDetectionThresholds(free_to_fixed_threshold, free_to_free_threshold);
```
The `free_to_fixed_threshold` and `free_to_free_threshold` refer to the detection thresholds for manipulators and static objects, and manipulators to other manipulators, respectively.

### Runtime
During the main loop, the positions of the free entities in the scene need to be updated. This can be done through the `UpdateVertices` function:
```c++
// Update the collision points for each of the free entities. The number of collision points must match the number set at each entity's construction. The order in which the entities are added to the scene become the indices for the first argument to UpdateVertices
scene.UpdateVertices(0, std::vector<Eigen::Vector3d>{
    left_point_1, left_point_2, left_point_3
});
scene.UpdateVertices(1, std::vector<Eigen::Vector3d>{
    right_point_1, right_point_2, right_point_3
});
```

Next, the potential collisions can be computed by simply calling `ComputeCollisions`. If visualization is desired, the locations and directions of these collisions can be retrieved from `GetCollisions`:
```c++
// Compute new potential collisions
scene.ComputeCollisions();

// Optional: the collision locations and directions can be retrieved for visualization
const gtfo::collision::Collision<double> left_collisions = scene.GetCollisions(0);
const gtfo::collision::Collision<double> right_collisions = scene.GetCollisions(1);
```

If virtual dynamics have been assosciated with any of the free entities, the dynamics need to be prevented from moving in the direction of collisions. This is simply done by calling a single function:
```c++
scene.UpdateVirtualState();
```
The function calls the corresponding `UpdateVirtualState` for each of the free entities added to the scene, which automatically updates their associated virtual dynamics accordingly.

# Contributing
We welcome all contributions, suggestions, and questions about the library. Please feel free to contact us directly through GitHub. 

Please report bugs through the [issue tracker](https://github.com/jianwei-sun/gtfo/issues).

When submitting pull requests, changes can be tested with the unit tests described below.
## Building and Running Tests
Tests and be built with the following commands:
```
mkdir build
cd build
cmake ..
cmake --build . -j8
```
The `8` can be changed to any positive integer to denote the number of parallel jobs. Note that if your third-party dependencies are installed in non-default locations, you can specify them when you first call `cmake` with:
```
cmake .. -DCMAKE_PREFIX_PATH=<your first location>:<your second location>:<etc>
```
After building, tests can then be executed with the following command in the `build` directory:
```
ctest
```
To run a specific testsuite (`NormBoundTest` for example) and output upon failure, run:
```
ctest -R NormBoundTest --output-on-failure
```
A specific test within the testsuite can also be run with:
```
ctest -R NormBoundTest.Contains1D --output-on-failure
``` 

# License and Disclaimer
Copyright 2023 UCLA Samueli School of Engineering. Developed by the [Bionics Lab](http://bionics.seas.ucla.edu/index.html).

