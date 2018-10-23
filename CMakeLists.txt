cmake_minimum_required(VERSION 3.5.1)
project(PclSegmentation)
add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 11)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 QUIET)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(PclSegmentation main.cpp)

target_link_libraries(PclSegmentation PCL)
