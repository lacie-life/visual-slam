cmake_minimum_required(VERSION 3.19)
project(geometry)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

# for visual trajectory example
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable(useEigen useEigen.cpp)
target_link_libraries( useEigen Eigen3::Eigen)

add_executable(useGeometry useGeometry.cpp)
target_link_libraries(useGeometry Eigen3::Eigen)

add_executable(example-1 example-1.cpp)
target_link_libraries(example-1 Eigen3::Eigen)

add_executable( trajectory plotTrajectory.cpp )
target_link_libraries( trajectory ${Pangolin_LIBRARIES} Eigen3::Eigen)

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} Eigen3::Eigen)

