cmake_minimum_required(VERSION 2.8.3)
project(vtec_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
	FILES
	TrackingResult.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs 
	geometry_msgs 
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

