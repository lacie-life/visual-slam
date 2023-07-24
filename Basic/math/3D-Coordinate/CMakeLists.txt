cmake_minimum_required(VERSION 3.2)
project(3dcoordinate)
set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(STATUS "Eigen3 found" ${EIGEN3_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
message(STATUS "PANGOLIN_INCLUDE_DIRS found" ${Pangolin_INCLUDE_DIRS})
message(STATUS "Pangolin_LIBRARIES found" ${Pangolin_LIBRARIES})

add_executable(eigenMatrix  eigenMatrix.cpp)
add_executable(coordinateTransform coordinateTransform.cpp)

add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})

