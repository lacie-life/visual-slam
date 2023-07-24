cmake_minimum_required(VERSION 3.19)
# Create Project
project(rgbd-3)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )


ADD_EXECUTABLE( generate_pointcloud src/generatePointCloud.cpp  )
TARGET_LINK_LIBRARIES( generate_pointcloud
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )

ADD_LIBRARY( slambase SHARED src/slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )

ADD_EXECUTABLE( detectFeatures src/detectFeatures.cpp )
TARGET_LINK_LIBRARIES( detectFeatures
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )

ADD_EXECUTABLE( featureExtract src/main.cpp )
TARGET_LINK_LIBRARIES( featureExtract
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )