cmake_minimum_required(VERSION 2.8)
project(rvio_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(rvio_header_dir
    src/include
    )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport)

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)

FILE(GLOB_RECURSE rvio_source_files
    ${PROJECT_SOURCE_DIR}/src/source/*.cpp
    )

FILE(GLOB_RECURSE rvio_header_files
    ${PROJECT_SOURCE_DIR}/src/include/*.h
    )

find_package(Eigen3)

catkin_package(INCLUDE_DIRS ${rvio_header_dir}
  LIBRARIES rvio_lib
  CATKIN_DEPENDS roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  camera_models
  image_transport)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${rvio_header_dir}
  ${GTSAM_INCLUDE_DIR}
)

add_library(rvio_lib ${rvio_source_files} ${rvio_header_files})
target_link_libraries(rvio_lib 
		      ${catkin_LIBRARIES} 
		      ${OpenCV_LIBS}
		      ${Eigen_INCLUDE_DIRS}
                      gtsam
                      gtsam_unstable
		      )

add_executable(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/src/rvio_node.cpp)
target_link_libraries(${PROJECT_NAME}_node rvio_lib)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_test ${PROJECT_SOURCE_DIR}/src/test/test_graph.cpp)
target_link_libraries(${PROJECT_NAME}_test rvio_lib)
add_dependencies(${PROJECT_NAME}_test ${catkin_EXPORTED_TARGETS})
