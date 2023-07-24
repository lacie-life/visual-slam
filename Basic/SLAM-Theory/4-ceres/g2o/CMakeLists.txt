cmake_minimum_required( VERSION 3.2 )
project (g2o-example)
set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

FIND_PACKAGE(G2O REQUIRED)

find_package(OpenCV REQUIRED)
# Eigen
include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
)
add_executable(g2o-example g2o-example.cpp)
target_link_libraries(g2o-example ${OpenCV_LIBS}  
                                  ${CSPARSE_LIBRARY} 
                                  ${G2O_LIBS}
                                  ${G2O_CORE_LIBRARY} 
                                  ${G2O_STUFF_LIBRARY})
target_link_libraries( g2o-example Eigen3::Eigen)