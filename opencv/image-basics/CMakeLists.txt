cmake_minimum_required(VERSION 3.0)

project(imageBasics)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )

add_executable(imageBasics imageBasics.cpp)
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
target_link_libraries(undistortImage ${OpenCV_LIBS})