# Generated Trajectories for Orthoses (GTFO)

`gtfo` is a virtual dynamics and trajectory generation library for physical human-robot interaction applications. This library serves to provide an admittance control layer for basic physical interaction, and allows for more complex applications to be built on top. The library simulates virtual first- and second-order dynamics for point masses and rigid bodies. Safety features, such as collision and boundary avoidance, are built-in and can be set as desired. 

Written by the Bionics Lab at UCLA. This library is currently under development!

## Initializing Submodules
This project includes `Eigen` and `MuJoCo` as git submodules. To populate these dependencies, run the following command the first time:
```
git submodule update --init --recursive
```

## Building Third-party Libraries
As `Eigen` is a header-only, only `MuJoCo` needs to be built before using. In the `External/mujoco` directory:
```
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../bin
cmake --build . -j4
cmake --install .
```

## Building Tests
Tests and be built with the following commands:
```
mkdir build
cd build
cmake ..
cmake --build . -j4
```
They can then be executed with the following command in the `build` directory:
```
ctest
```