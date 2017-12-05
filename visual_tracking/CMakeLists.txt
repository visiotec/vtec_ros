cmake_minimum_required(VERSION 2.8.3)
project(visual_tracking)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	cv_wrapper
	roscpp
	image_transport
	vtec_msgs
)

find_package(vtec)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES visual_tracking
	CATKIN_DEPENDS roscpp 
	DEPENDS vtec
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${vtec_INCLUDE_DIRS}
)

add_executable(tracker_node src/tracker_node.cpp)
target_link_libraries(tracker_node ${catkin_LIBRARIES} ${vtec_LIBRARIES})
