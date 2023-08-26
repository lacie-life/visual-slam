cmake_minimum_required(VERSION 3.26)
project(SimpleSfM)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
)

add_subdirectory(thirdparty)

file(GLOB HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/Feature/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/SfM/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/Optimizer/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/Utils/*.h
)

file(GLOB SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Feature/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/SfM/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Optimizer/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Utils/*.cpp
)

add_library(SimpleSfM_lib
        ${HEADERS}
        ${SOURCES}
)

target_link_libraries(SimpleSfM_lib
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    sqlite3
    -lpthread
)

add_executable(SimpleSfMPipeline app/main.cpp)

target_link_libraries(SimpleSfMPipeline
    SimpleSfM_lib
    ${OpenCV_LIBS}
    sqlite3
    -lpthread
)

add_executable(FeatureExtraction app/FeatureExtraction.cpp)
target_link_libraries(FeatureExtraction
        SimpleSfM_lib
        ${OpenCV_LIBS}
        sqlite3
        -lpthread
)

add_executable(ComputeMatches app/ComputeMatches.cpp)
target_link_libraries(ComputeMatches
        SimpleSfM_lib
        ${OpenCV_LIBS}
        sqlite3
        -lpthread
)

add_executable(CheckMatches app/CheckMatches.cpp)
target_link_libraries(CheckMatches
        SimpleSfM_lib
        ${OpenCV_LIBS}
        sqlite3
        -lpthread
)

add_executable(Reconstruction app/Reconstruction.cpp)
target_link_libraries(Reconstruction
        SimpleSfM_lib
        ${OpenCV_LIBS}
        sqlite3
        -lpthread
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


