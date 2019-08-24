cmake_minimum_required(VERSION 2.8.3)
project(vtec_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	image_transport
	vtec_msgs
	std_msgs
	cv_bridge 
)

find_package(vtec REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS xfeatures2d)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp 
	DEPENDS vtec
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${vtec_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

message(${vtec_LIBRARIES})

add_executable(ibgho_tracker_node src/ibgho_tracker_node.cpp)
target_link_libraries(ibgho_tracker_node ${catkin_LIBRARIES} ${vtec_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(unified_tracker_node src/unified_tracker_node.cpp)
target_link_libraries(unified_tracker_node ${catkin_LIBRARIES} ${vtec_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(keyboard_control src/key_commands.cpp)
target_link_libraries(keyboard_control ${catkin_LIBRARIES})

