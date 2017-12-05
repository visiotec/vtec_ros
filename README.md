# vtec_ros #

[ROS] packages from the VisioTec group

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

* **`reference_image`** ([sensor_msgs/Image])

   The reference template extracted from the reference image file.   

* **`tracking`** ([visual_tracking/TrackingResult])

   Information about the tracking. Includes the estimated homography and the photometric parameters.

#### Parameters

* **`image_topic`** (string, default: "camera/image")

   The name of the image input topic.

* **`bbox_pos_x`** (int, default: 200)

   The x coordinate of the upper left corner of the region of interest in the reference image.

* **`bbox_pos_y`** (int, default: 150)

   The y coordinate of the upper left corner of the region of interest in the reference image.

* **`bbox_size_x`** (int, default: 200)

   The length in pixels of the region of interest along the x direction.

* **`bbox_size_x`** (int, default: 200)

   The length in pixels of the region of interest along the y direction.

* **`max_nb_iter_per_level`** (int, default: 5)

   Maximum number of optimization iterations per pyramid level.

* **`max_nb_pyr_level`** (int, default: 2)

   Maximum number of pyramids levels.

* **`sampling_rate`** (double, default: 1.0)

   The sampling rate used to sample points used in the optimization process. 1.0 means 100% of the points are used.

* **`reference_image_path`** (string, "$(find visual\_tracking)/imgs/ref\_img.png")

   Path to the reference image.

## Usage ##

### Runnning with a dataset ###

Download the dataset from here: [dataset](https://www.dropbox.com/s/uhzg6rlk92zzxou/vtec_tracker_test.bag?dl=0)

Open a terminal window and launch the tracker node with:

```
roslaunch visual_tracking tracker_live.launch
```

In another terminal, play the downloaded bagfile with:

```
rosbag play vtec_tracker_test.bag
```

Now you should see in RViz the tracking process.

### Running from a live camera ###

The tracker node needs a reference image. The images are located in the [imgs](visual_tracking/imgs) folder. To start using the node, you can either use the provided [ref_img.png](visual_tracking/imgs/ref_img.png) file, or use your own. The location of the reference image should be set using the `reference_image_path` parameter. Print the reference image so you can start playing with our tracker.

After correctly selecting the reference image and setting the bounding box parameters, open a terminal window and launch the tracker node:

```
roslaunch visual_tracking tracker_live.launch
```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/lukscasanova/vtec_ros/issues).


[ROS]: http://www.ros.org
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[visual_tracking/TrackingResult]: visual_tracking/msg/TrackingResult.msg

