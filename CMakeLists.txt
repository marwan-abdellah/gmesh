# CMake simple example

## [main]

# Almost all CMake files should start with this
# You should always specify a range with the newest
# and oldest tested versions of CMake. This will ensure
# you pick up the best policies.
cmake_minimum_required(VERSION 3.1...3.27)

# This is your project statement. You should always list languages;
# Listing the version is nice here since it sets lots of useful variables
project(
  gmesh
  VERSION 1.0
  LANGUAGES CXX)

# If you set any CMAKE_ variables, that can go here.
# (But usually don't do this, except maybe for C++ standard)

# Find packages go here.

# You should usually split this into folders, but this is a simple example

# This is a "default" library, and will match the *** variable setting.
# Other common choices are STATIC, SHARED, and MODULE
# Including header files here helps IDEs but is not required.
# Output libname matches target name, with the usual extensions on your system
add_library(gmesh
    src/simple_lib.cpp
    src/simple_lib.hpp
    src/Common.hh
    src/Vertex.hpp
    src/Triangle.hpp
    src/EigenVector.hpp
    src/Neighbour.hpp
    src/SurfaceMesh.cc
    src/SurfaceMesh.hh
    src/SurfaceMesh.hpp
    src/SurfaceMeshOperations.cc
    src/SurfaceSubdivision.cc
    src/SurfaceMeshSmooth.cc
    src/SurfaceMeshCoarse.cc
)

# Link each target with other targets or add options, etc.

# Adding something we can run - Output name matches target name
add_executable(gmesh_example
    example/simple_example.cpp)

# Make sure you link your targets with this command. It can also link libraries and
# even flags, so linking a target that does not exist will not give a configure-time error.
target_link_libraries(gmesh_example PRIVATE gmesh)



