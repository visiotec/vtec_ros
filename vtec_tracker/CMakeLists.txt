cmake_minimum_required(VERSION 2.8.3)
project(vtec_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	cv_wrapper
	roscpp
	image_transport
	vtec_msgs
	std_msgs
)

find_package(vtec)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp 
	DEPENDS vtec
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${vtec_INCLUDE_DIRS}
)

add_executable(ibgho_tracker_node src/ibgho_tracker_node.cpp)
target_link_libraries(ibgho_tracker_node ${catkin_LIBRARIES} ${vtec_LIBRARIES})

add_executable(keyboard_control src/key_commands.cpp)
target_link_libraries(keyboard_control ${catkin_LIBRARIES})

