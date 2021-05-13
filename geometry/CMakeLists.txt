cmake_minimum_required(VERSION 3.19)
project(geometry)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

add_executable(useEigen useEigen.cpp)
target_link_libraries( useEigen Eigen3::Eigen)

add_executable(useGeometry useGeometry.cpp)
target_link_libraries(useGeometry Eigen3::Eigen)
