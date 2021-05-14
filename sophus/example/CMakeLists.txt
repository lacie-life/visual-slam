cmake_minimum_required(VERSION 3.0)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/groundtruth.txt
     DESTINATION   ${CMAKE_CURRENT_BINARY_DIR}/data/groundtruth.txt)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/estimated.txt
     DESTINATION   ${CMAKE_CURRENT_BINARY_DIR}/data/estimated.txt)
