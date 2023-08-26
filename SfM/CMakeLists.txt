cmake_minimum_required(VERSION 3.26)
project(SimpleSfM)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

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
    sqlite3
)

add_executable(SimpleSfMPipeline test/main.cpp)

target_link_libraries(SimpleSfMPipeline
    SimpleSfM_lib
    ${OpenCV_LIBS}
    sqlite3
)




