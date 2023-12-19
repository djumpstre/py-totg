cmake_minimum_required(VERSION 3.5)
project(py_totg)
set(CMAKE_CXX_STANDARD 11)

# find_package(pybind11 2.9 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)  # Set path that includes eigen3config.cmake.
include_directories(${EIGEN3_INCLUDE_DIR})

message("${CMAKE_MODULE_PATH}")
message("${EIGEN3_INCLUDE_DIR}")

pybind11_add_module(py_totg src/py_totg.cpp
                            src/Trajectory.cpp
                            src/Path.cpp)

add_executable(test_example src/Example_9dof.cpp
                            src/Trajectory.cpp
                            src/Path.cpp)

