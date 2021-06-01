project(Factor_lib)

set(HEADER_FILES PointToPlaneFactor.h LBLFactor.h PointToPlaneFactor2.h PointToPlaneFactor3.h)
set(SOURCE_FILES PointToPlaneFactor.cpp LBLFactor.cpp PointToPlaneFactor2.cpp PointToPlaneFactor3.cpp)

include_directories("/usr/include/eigen3")

find_package(GTSAM REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})
add_library(Factor_lib STATIC ${HEADER_FILES} ${SOURCE_FILES})

