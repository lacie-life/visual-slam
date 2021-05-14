cmake_minimum_required(VERSION 3.0)

project(stereoVision)

find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/images/left.png
     DESTINATION   ${CMAKE_CURRENT_BINARY_DIR}/images/left.png)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/images/right.png
     DESTINATION   ${CMAKE_CURRENT_BINARY_DIR}/images/right.png)