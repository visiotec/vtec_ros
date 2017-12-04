# vtec_ros #

ROS packages from the VisioTec group

## Installation ##

Install the usb_cam driver from ROS repositories.

```
sudo apt-get install ros-kinetic-usb-cam
```

Setup a ROS workspace.

```
cd 
mkdir -p catkin_ws/src
```

Install the vtec cpp library

```
cd ~/catkin_ws/src
git clone https://github.com/lukscasanova/vtec_cpp_release.git
cd vtec_cpp_release
mkdir build
cd build
cmake ..
make
```

Install the ROS packages

```
cd ~/catkin_ws/src
git clone https://github.com/lukscasanova/vtec_ros
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Running ##

Launch the tracker node with:

```
roslaunch visual_tracking tracker.launch
```




