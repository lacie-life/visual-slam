cmake_minimum_required(VERSION 2.8)
project(ceres-example)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)

add_executable(ceres-example ceres-example.cpp)
target_link_libraries(ceres-example ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries( ceres-example Eigen3::Eigen)
