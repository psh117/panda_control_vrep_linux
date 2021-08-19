# Panda Control in V-REP (CoppeliaSim)

## Requirements
- CoppeliaSim 4.0.0
- CMake
- RBDL (https://github.com/ORB-HD/rbdl-orb)

## Installation and Execution

```sh
mkdir build
cd build
cmake ..
make
```
- make sure you've enabled shared memory control setting in v-rep (coppeliasim). 
the port is set to -3
- make sure v-rep (coppeliasim) scene is loaded and ready to move before execution
```sh
./panda_control_vrep
````