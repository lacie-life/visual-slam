cmake_minimum_required(VERSION 3.2)
project(poseEstimationExample)
set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")

# CMAKE_CXX_STANDARD 14
 set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
#standards c14
#set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV found: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV library: ${OpenCV_LIBS}")

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(imageMatching imageMatching.cpp)
target_link_libraries(imageMatching ${OpenCV_LIBS})

add_executable(pose_est_2d2d poseEstimation2d2d.cpp)
target_link_libraries(pose_est_2d2d ${OpenCV_LIBS})
