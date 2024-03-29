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

# download opencv4.2 
# https://github.com/opencv/opencv/releases/tag/4.2.0
# https://github.com/opencv/opencv_contrib/releases/tag/4.2.0
find_package(OpenCV 4.2.0 REQUIRED )
find_package(OpenCV 4.2.0 REQUIRED COMPONENTS xfeatures2d)
message( "Opencv version found: " ${OpenCV_VERSION} )
message( "Opencv libraries found: " ${OpenCV_LIBRARIES} )

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


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE )
install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE )
install(DIRECTORY config/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE )

message(${vtec_LIBRARIES})

add_executable(ibgho_tracker_node src/ibgho_tracker_node.cpp)
target_link_libraries(ibgho_tracker_node ${catkin_LIBRARIES} ${vtec_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(unified_tracker_node src/unified_tracker_node.cpp)
target_link_libraries(unified_tracker_node ${catkin_LIBRARIES} ${vtec_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(keyboard_control src/key_commands.cpp)
target_link_libraries(keyboard_control ${catkin_LIBRARIES})

install(TARGETS keyboard_control unified_tracker_node ibgho_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
