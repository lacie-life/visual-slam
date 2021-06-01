cmake_minimum_required(VERSION 2.6)
project(IMUKittiExampleGPS)

find_package(GTSAM REQUIRED)
include_directories(
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	${GTSAM_LIBRARY_DIRS}
)

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(IMUKittiExampleGPS src/IMUKittiExampleGPS.cpp)
target_link_libraries(IMUKittiExampleGPS gtsam ${Boost_LIBRARIES})
