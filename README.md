# Panda Control in V-REP (CoppeliaSim)

## Requirements
- CoppeliaSim 4.0.0
- CMake
- RBDL (https://github.com/ORB-HD/rbdl-orb)

## Installation and Execution
```sh
git clone --recursive https://github.com/psh117/panda_control_vrep_linux
cd panda_control_vrep_linux
mkdir build
cd build
cmake ..
make
```
- make sure you've enabled shared memory control setting in v-rep (coppeliasim). 
the port is set to -3
( remoteApiConnections.txt -> portIndex1_port = -3 )
- make sure v-rep (coppeliasim) scene is loaded and ready to move before execution
```sh
./panda_control_vrep
````
