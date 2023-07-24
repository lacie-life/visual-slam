cmake_minimum_required(VERSION 3.2)
project(ch04)

set(CMAKE_CXX_STANDARD 14)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
#include_directories(${Sophus_INCLUDE_DIRS})

message(STATUS "EIGEN3_INCLUDE_DIRS found" ${EIGEN3_INCLUDE_DIRS})


add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)
