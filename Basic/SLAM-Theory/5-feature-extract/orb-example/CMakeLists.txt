cmake_minimum_required(VERSION 2.8)

project(orb-example)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV 3 REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable(orb_example orb.cpp)
target_link_libraries(orb_example ${OpenCV_LIBS})