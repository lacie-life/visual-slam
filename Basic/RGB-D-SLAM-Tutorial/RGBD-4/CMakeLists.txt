cmake_minimum_required(VERSION 3.19)
# Create Project
project(RGBD-4)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

ADD_LIBRARY( slambase SHARED src/slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )

ADD_EXECUTABLE( joinPointCloud src/main.cpp )
TARGET_LINK_LIBRARIES( joinPointCloud
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )
