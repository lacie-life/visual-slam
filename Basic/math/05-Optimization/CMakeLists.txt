cmake_minimum_required(VERSION 3.2)
project(optimizationExample)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3: ${EIGEN3_VERSION}")
message(STATUS "Found EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED)
#message opencv include dir
message(STATUS "Found OpenCV: ${OPENCV_VERSION}")
message(STATUS "Found OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "Found OpenCV_LIBRARIES: ${OpenCV_LIBS}")

find_package(Ceres REQUIRED)
message(STATUS "Found Ceres: ${CERES_VERSION}")
message(STATUS "Found Ceres_INCLUDE_DIRS: ${CERES_INCLUDE_DIRS}")

find_package(G2O REQUIRED)
message(STATUS "Found G2O: ${G2O_VERSION}")
message(STATUS "Found G2O_INCLUDE_DIRS: ${G2O_INCLUDE_DIRS}")
message(STATUS "Found G2O_CORE_LIBRARY: ${G2O_CORE_LIBRARY}")
message(STATUS "Found G2O_STUFF_LIBRARY: ${G2O_STUFF_LIBRARY}")

find_package(GTSAM REQUIRED)
message(STATUS "Found gtsam: ${GTSAM_VERSION}")
message(STATUS "Found gtsam_INCLUDE_DIRS: ${GTSAM_INCLUDE_DIR}")
set(GTSAM_LIBRARIES gtsam) # gtsam_core gtsam_stuff
message(STATUS "Found gtsam_LIBRARIES: ${GTSAM_LIBRARIES}")
include_directories(${GTSAM_INCLUDE_DIR})

include_directories(${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})

add_executable(gaussNewton gaussNewton.cpp)

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${G2O_LIBRARIES} ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

add_executable(gtsamCurveFitting gtsamCurveFitting.cpp)
target_link_libraries(gtsamCurveFitting ${GTSAM_LIBRARIES})