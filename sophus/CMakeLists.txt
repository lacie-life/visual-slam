cmake_minimum_required(VERSION 3.0)
project(sophus-example)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} ) # sohpus is header only

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)
