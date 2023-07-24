cmake_minimum_required(VERSION 3.2)
project(EvaluateError)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
message(STATUS "Pangolin found" ${Pangolin_INCLUDE_DIRS})

add_executable(evaluation evaluateError.cpp)
target_link_libraries(evaluation ${Pangolin_LIBRARIES} Sophus::Sophus)
message(STATUS "Libraries found " ${Pangolin_LIBRARIES})
