hri_skeletons
=============

A ROS node that detects and track human bodies in both 2D (2D keypoints of the 
body, in the image space) and 3D (generating on the fly URDF models for each 
detected humans, alongside their joint state and TF trees).

The detection relies on the open-source [Intel OpenVINO toolkit](https://docs.openvinotoolkit.org/),
requires only a RGB stream, and runs at ~15FPS on an Intel 10th gen CPU.


Installation
------------

- Follow first the general [ROS4HRI installation instructions].

- Install the `hri_msgs` package

- then:

```
$ cd ~/src
$ git clone https://git.brl.ac.uk/ROS4HRI/hri_skeletons.git
$ cd hri_skeletons
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/dev .. && make && make install
```

You can test this node by running:
```
rosrun hri_skeletons human_pose_estimation_3d.py image:=/usb_cam/image_raw _debug:=true _model:=/home/severin/openvino_models/public/human-pose-estimation-3d-0001/FP16/human-pose-estimation-3d-0001.xml
```

which should display something like that (debug mode = true):

![Screenshot_from_2020-09-25_22-44-09](uploads/dd959478a1f8c2a7fed7c47a822ca7b5/Screenshot_from_2020-09-25_22-44-09.png)