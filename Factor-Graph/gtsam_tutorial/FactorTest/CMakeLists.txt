cmake_minimum_required(VERSION 3.15)
project(FactorTest)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native ")
set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")

find_package(Eigen3 REQUIRED)

set(SOURCE_FILES main.cpp)
add_executable(FactorTest_run main.cpp)

include_directories(Factor_lib)
add_subdirectory(Factor_lib)

target_link_libraries(FactorTest_run Factor_lib gtsam -lboost_system)
add_subdirectory(gtests)