The majority of the the external libraries that support MACE are housed in the tools folder of this project. They should be built in the following order to ensure that the dependencies are met. First, you should sync with the submodules if MACE has not been pulled recursively. This can be accomplished by running the following commands:

In Windows the FCL tool requires MinGW64 to compile successfully.
See instructions [Here](https://github.com/heronsystems/OpenMACE/wiki/Installing-MinGW-on-Windows) for how to install MinGW in Windows. Change directories into the `MACE/tools` directory and initialize the git submodules.

```
$ cd $MACE_ROOT/tools
$ git submodule init
$ git submodule update
```

# FLANN - Fast Library for Approximate Nearest Neighbors

FLANN is a library for performing fast approximate nearest neighbor searches in high dimensional spaces. It
contains a collection of algorithms we found to work best for nearest neighbor search and a system for
automatically choosing the best algorithm and optimum parameters depending on the dataset.

Library Git Location:
Library URL Location:
## UBUNTU INSTALL
```
$ cd $MACE_ROOT/tools/flann/
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

## Windows Install with MinGW

Ensure that the MinGW directory on the PATH is the same directory used in the QT kit using to compile MACE

```
$ cd $MACE_ROOT/tools/flann/
$ mkdir build
$ cd build
$ cmake .. -G "MinGW Makefiles"
$ mingw32-make
```

No need to move artifacts as MACE is configured to look in the *tools/* directory.

# OCTOMAP - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
The OctoMap library implements a 3D occupancy grid mapping approach, providing data structures and mapping algorithms in C++ particularly suited for robotics.

Library Git Location: https://github.com/OctoMap/octomap

Library URL Location: https://octomap.github.io/

## UBUNTU INSTALL
```
$ cd $MACE_ROOT/tools/octomap/
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

## Windows Install with MinGW

Ensure that the MinGW directory on the PATH is the same directory used in the QT kit using to compile MACE

```
$ cd $MACE_ROOT/tools/octomap/
$ mkdir build
$ cd build
$ cmake .. -G "MinGW Makefiles"
$ mingw32-make
```

No need to move artifacts as MACE is configured to look in the *tools/* directory.

# LIBCCD - Library For Collision Detection Between Two Convex Shapes
libccd is library for a collision detection between two convex shapes. libccd implements variation on Gilbert–Johnson–Keerthi algorithm plus Expand Polytope Algorithm (EPA) and also implements algorithm Minkowski Portal Refinement (MPR, a.k.a. XenoCollide) as described in Game Programming Gems 7.

Library Git Location: https://github.com/danfis/libccd

Library URL Location:

## UBUNTU INSTALL
First, we need to specify the definition of whether we are going to be using SINGLE or DOUBLE point floating precision. For some reason, the CMAKEFILE does not handle this, thus for now we need to correct the vec3.h file. To do this perform the following:
```
$ cd $MACE_ROOT/tools/libccd/src/ccd
$ gedit vec3.h
At this stage add **#define CCD_SINGLE** or **#define CCD_DOUBLE** below the include statements per the type of floating precision desired.
```
Save and close the file.
```
$ cd $MACE_ROOT/tools/libccd
$ mkdir build
$ cd build
$ cmake -G "Unix Makefiles" ..
$ make
$ sudo make install
```

# FCL - The Flexible Collision Library
FCL is a library for performing three types of proximity queries on a pair of geometric models composed of triangles.
* Collision detection: detecting whether the two models overlap, and optionally, all of the triangles that overlap.
* Distance computation: computing the minimum distance between a pair of models, i.e., the distance between the closest pair of points.
* Tolerance verification: determining whether two models are closer or farther than a tolerance distance.
* Continuous collision detection: detecting whether the two moving models overlap during the movement, and optionally, the time of contact.
* Contact information: for collision detection and continuous collision detection, the contact information (including contact normals and contact points) can be returned optionally.


Library Git Location: https://github.com/flexible-collision-library/fcl

Library URL Location:

Library API Documentation: https://flexible-collision-library.github.io/
## UBUNTU INSTALL
```
$ cd $MACE_ROOT/tools/fcl
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

## Windows MinGW Install

Open *tools/fcl/src/CMakeList.txt* and change the following lines:
```
if(TARGET ccd)
  target_link_libraries(${PROJECT_NAME} PUBLIC ccd)
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC ccd)
  #target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CCD_INCLUDE_DIRS}")
  #target_link_libraries(${PROJECT_NAME} PUBLIC "${CCD_LIBRARIES}")
endif()
```

In FCL folder with MinGW64 tools run the following commands:
```
mkdir build
cd build
cmake -G "MSYS Makefiles" -DCMAKE_CXX_FLAGS="-L\"C:/Program Files (x86)/libccd/bin\" -I \"C:/Program Files (x86)/libccd/include\" -Wa,-mbig-obj" -DCCD_INCLUDE_DIR="C:/Program Files (x86)/libccd/include" -DCCD_LIBRARY="C:/Program Files (x86)/libccd/bin/libccd.dll" ..
make
```

MAKE will probably fail when compiling test. They can be ignored as it's unneeded, but it will prevent `make install` from working properly. You may need to browse to build directory and move the artifacts yourself.