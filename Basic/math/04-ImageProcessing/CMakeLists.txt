cmake_minimum_required(VERSION 3.2)
project(imageExample)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

#show eigen3 include path
message(STATUS "Eigen3 include path: ${EIGEN3_INCLUDE_DIRS}")
include_directories(${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_subdirectory(BasicImage)
add_subdirectory(stereo)
add_subdirectory(rgbd)

