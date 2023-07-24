cmake_minimum_required(VERSION 3.21)
project(imageBasic)
add_executable(imageBasic imageBasic.cpp)
target_link_libraries(imageBasic ${OpenCV_LIBRARIES})

add_executable(undistortImg undistortImg.cpp)
target_link_libraries(undistortImg ${OpenCV_LIBRARIES})