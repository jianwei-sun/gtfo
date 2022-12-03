# Generated Trajectories for Orthoses (GTFO)

`gtfo` is a virtual dynamics and trajectory generation library for physical human-robot interaction applications. This library serves to provide an admittance control layer for basic physical interaction, and allows for more complex applications to be built on top. The library simulates virtual first- and second-order dynamics for point masses and rigid bodies. Safety features, such as collision and boundary avoidance, are built-in and can be set as desired. 

Written by the Bionics Lab at UCLA. This library is currently under development!

## Building Tests
Tests and be built with the following commands:
```
mkdir build
cd build
cmake -G "Ninja" ..
ninja -j4
```
They can then be executed with the following command in the `build` directory:
```
ctest
```