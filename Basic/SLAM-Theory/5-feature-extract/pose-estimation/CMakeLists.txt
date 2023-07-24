cmake_minimum_required(VERSION 2.8)
project(pose-estimation)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV 3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

# add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp extra.cpp ) # use this if in OpenCV2 
add_executable(2d2d 2d2d.cpp)
target_link_libraries(2d2d ${OpenCV_LIBS})

# # add_executable( triangulation triangulation.cpp extra.cpp) # use this if in opencv2
add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})

add_executable(3d2d 3d2d.cpp)
target_link_libraries(3d2d
                            ${G2O_LIBS}
                            ${G2O_CORE_LIBRARY} 
                            ${G2O_STUFF_LIBRARY}
                            ${OpenCV_LIBS})

add_executable(3d3d 3d3d.cpp)
target_link_libraries(3d3d
                        ${G2O_LIBS}
                        ${G2O_CORE_LIBRARY} 
                        ${G2O_STUFF_LIBRARY}
                        ${OpenCV_LIBS})