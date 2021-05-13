cmake_minimum_required(VERSION 3.19)
project(useEigen)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries( ${PROJECT_NAME}  Eigen3::Eigen)