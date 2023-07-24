cmake_minimum_required(VERSION 2.6)
project(linefitting)

find_package(GTSAM REQUIRED)
include_directories(
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	${GTSAM_LIBRARY_DIRS}
)

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(linefitting src/linefitting.cpp)
target_link_libraries(linefitting gtsam ${Boost_LIBRARIES})

add_executable(linefittingExpressions src/linefittingExpressions.cpp)
target_link_libraries(linefittingExpressions gtsam ${Boost_LIBRARIES})
