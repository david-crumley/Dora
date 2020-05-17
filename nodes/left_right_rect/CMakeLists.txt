cmake_minimum_required(VERSION 2.8.3)
project(left_right_rect)

# if CMake_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
)


catkin_package()

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

## Build
add_executable(left_right_rect src/left_right_rect_sub.cpp)
target_link_libraries(left_right_rect ${catkin_LIBRARIES})
