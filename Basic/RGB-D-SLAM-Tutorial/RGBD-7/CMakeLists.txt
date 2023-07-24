cmake_minimum_required(VERSION 3.10)
project(RGBD_7)

set(CMAKE_CXX_STANDARD 14)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Add dependency of g2o
# Because g2o is not a common library, add its findg2o.cmake file

FIND_PACKAGE( G2O REQUIRED)
# CSparse
FIND_PACKAGE( CSparse REQUIRED)
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY( slambase SHARED src/slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )

ADD_EXECUTABLE( slamEnd src/main.cpp )
TARGET_LINK_LIBRARIES( slamEnd
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${G2O_LIBS}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        g2o_types_slam3d
        cholmod
        ${CSPARSE_LIBRARY})