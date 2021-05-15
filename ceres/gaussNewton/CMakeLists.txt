cmake_minimum_required(VERSION 2.8)
project(gaussNewton)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)

add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBS} )
target_link_libraries( gaussNewton Eigen3::Eigen)
