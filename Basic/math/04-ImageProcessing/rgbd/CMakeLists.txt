find_package(Sophus REQUIRED)
set(CMAKE_CXX_STANDARD 14)

message(STATUS "Found Sophus")
# sophus include dir
message(STATUS "Sophus include dir: ${Sophus_INCLUDE_DIRS}")
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

find_package(Open3D REQUIRED)


add_executable(rgbdMap rgbdMap.cpp)
target_link_libraries(rgbdMap ${Pangolin_LIBRARIES} ${OpenCV_LIBRARIES} Sophus::Sophus Open3D::Open3D)