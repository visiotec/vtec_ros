# vtec_ros #

ROS packages from the VisioTec group

## Installation ##

First, setup a ROS workspace.

``
cd 
mkdir -p catkin_ws/src
``

Then, install the vtec cpp library

``
cd ~/catkin_ws/src
git clone https://github.com/lukscasanova/vtec_cpp_release.git
cd vtec_cpp_release
mkdir build
cd build
cmake ..
make
``

Lastly, install the ROS packages

``
cd ~/catkin_ws/src
git clone https://github.com/lukscasanova/vtec_ros
cd ~/catkin_ws
catkin_make
``

## Running ##




