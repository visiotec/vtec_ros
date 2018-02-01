# VisioTec ROS Packages

[ROS] Kinetic packages developed at the VisioTec research group of the CTI Renato Archer. Further information about this group can be found [here](https://sites.google.com/site/geraldofsilveira/).


## Video Examples ##

Click on the thumbnails to watch the videos on YouTube.

* Intensity-based visual tracking with full 8-DoF homography

[![YouTube](https://img.youtube.com/vi/r7kZLqQ5xbI/0.jpg)](https://www.youtube.com/watch?v=r7kZLqQ5xbI)

* Intensity-based visual tracking with affine 6-DoF homography

[![YouTube](https://img.youtube.com/vi/W-7otD3THM4/0.jpg)](https://www.youtube.com/watch?v=W-7otD3THM4)


## Documentation and Citing ##

The technical report available [here](https://github.com/lukscasanova/vtec/blob/master/vtec_ibgho_TR.pdf) describes the underlying algorithm and its working principles. If you use this software, please cite the technical report using:

```
@TechReport{nogueira2017,
  author = {Lucas Nogueira and Ely de Paiva and Geraldo Silveira},
  title  = {{VISIOTEC} Intensity-based Homography Optimization Software: Basic Theory and Use Cases},
  number = {CTI-VTEC-TR-01-2017},
  institution = {CTI},
  year = {2017},
  address = {Brazil}
}
```

## Installation ##

### Dependencies ###

Install the usb_cam driver from ROS repositories.

```
sudo apt-get install ros-kinetic-usb-cam
```

### Build ###
Setup a ROS workspace.

```
mkdir -p ~/catkin_ws/src
```

Install the vtec cpp library

```
cd ~/catkin_ws/src
git clone https://github.com/lukscasanova/vtec.git
cd vtec
mkdir build
cd build
cmake ..
make
```

Install the ROS packages

```
cd ~/catkin_ws/src
git clone https://github.com/lukscasanova/vtec_ros.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```


## Nodes ##


### ibgho_tracker_node ###

Tracks a planar object in an image sequence.

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

* **`tracking`** ([vtec_tracker/TrackingResult])

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

* **`homography_type`** (string, default: "full")

   Specifies the type of homography to be considered by the optimization algorithm. The options are: "full", "affine" and "stretch".

## Usage ##

### Runnning with a dataset ###

Download the dataset from here: [dataset](https://www.dropbox.com/s/zy829r6nrdqh4y6/vtec_test_tracker.bag?dl=0)

Open a terminal window and launch the tracker node with:

```
roslaunch vtec_tracker tracker.launch
```

In another terminal, play the downloaded bagfile with:

```
rosbag play vtec_tracker_test.bag
```

Now you should see in RViz the tracking process.

### Running from a live camera ###

Open a terminal window and launch the tracker node:

```
roslaunch vtec_tracker tracker_live.launch
```

A Rviz window will pop-up with the camera images. In the terminal window where you issued the roslaunch command, press the **S** key to start tracking. This will select a bounding box in the current frame to be tracked. You can press **S** again anytime to restart the tracking process.

## Resources ##

* IBGHO Technical Report: [vtec\_ibgho\_TR.pdf](https://github.com/lukscasanova/vtec/blob/master/vtec_ibgho_TR.pdf)
* VisioTec C++ Libraries: [https://github.com/lukscasanova/vtec](https://github.com/lukscasanova/vtec)
* Geraldo Silveira's website: [https://sites.google.com/site/geraldofsilveira/](https://sites.google.com/site/geraldofsilveira/)


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/lukscasanova/vtec_ros/issues).

[ROS]: http://www.ros.org
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[vtec_tracker/TrackingResult]: vtec_tracker/msg/TrackingResult.msg

