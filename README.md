# vtec_ros #

ROS packages from the VisioTec group

## Installation ##

### Dependencies ###

Install the usb_cam driver from ROS repositories.

```
sudo apt-get install ros-kinetic-usb-cam
```


### Build ###
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

## Usage ##

Launch the tracker node with:

```
roslaunch visual_tracking tracker.launch
```


## Nodes ##


### tracker_node ###

Tracks a planar patch in a image.

#### Subscribed Topics

* **`camera/image`** ([sensor_msgs/Image])

   The incoming image stream from the camera.


#### Published Topics

* **`annotated_image`** ([sensor_msgs/Image])

   The image stream annotaded with the tracked image region and the score.


* **`stabilized_image`** ([sensor_msgs/Image])

   The warped image patch from the image stream, that tries to match to the reference image patch.


* **`tracking`** ([visual_tracking/TrackingResult])

   Information about the tracking. Includes the estimated homography and the photometric parameters.

#### Parameters

* **`image_topic`** (string, default: "/temperature")

   The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

   The size of the cache.
}


