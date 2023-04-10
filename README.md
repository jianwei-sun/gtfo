# Generated Trajectories for Orthoses (GTFO)

`gtfo` is a virtual dynamics and trajectory generation library for physical human-robot interaction applications. This library serves to provide an admittance control layer for basic physical interaction, and allows for more complex applications to be built on top. The library simulates virtual first- and second-order dynamics for point masses and rigid bodies. Safety features, such as collision and boundary avoidance, are built-in and can be set as desired. 

Written by the Bionics Lab at UCLA. This library is currently under development!

## Third-party Libraries
This project requires `Eigen` version `3.4.0`, `MuJoCo` version `2.3.1`, and `OSQP` version `0.6.2` as dependencies. Ensure that they are already installed on your computer. In addition, `osqp-cpp` and `abseil-cpp` are internal dependencies, which can be populated using the command below:
```
git submodule update --init --recursive
```

## Building and Running Tests
Tests and be built with the following commands:
```
mkdir build
cd build
cmake ..
cmake --build . -j8
```
Note that if your third-party dependencies are installed in non-default locations, you can specify them when you first call `cmake` with:
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
Note that tests can be skipped altogether by passing `-DGTFO_BUILD_TESTS=OFF` or setting `set(GTFO_BUILD_TESTS OFF)` in your `CMakeLists.txt`.