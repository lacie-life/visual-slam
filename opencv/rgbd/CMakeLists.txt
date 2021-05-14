cmake_minimum_required(VERSION 3.0)

project(joinMap)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES})