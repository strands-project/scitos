Openni_wrapper
===================

This package consists of a wrapper around the Openni2 RGBD camera drivers.

Launch files
=================

The openni_wrapper package can be launched in the following way:

```roslaunch openni_wrapper main.launch````


Image types
========================

Openni publishes depth images using uint16, aka the 16UC1 type in ros. Full discussion at: http://wiki.ros.org/depth_image_proc#depth_image_proc.2BAC8-convert_metric

The (depth) topics published by openni_wrapper have the following format:
* `/camera_namespace/depth/image_raw` - 16UC1
* `/camera_namespace/depth/image_rect` - 16UC1
* `/camera_namespace/depth/image_rect_meters` - 32FC1
* `/camera_namespace/depth_registered/image_rect` - 16UC1


Calibration parameters
=========================

Camera calibration parameters are published in the following topics (note that the calibration parameters are different for the two cameras):
* `/camera_namespace/rgb/camera_info`
* `/camera_namespace/depth/camera_info`

Launch parameters
===================

The openni_wrapper supports the following parameters:

* ```camera``` - the namespace of the first camera (e.g. head_xtion)
* ```camera2``` - the namespace of the second camera (e.g. chest_xtion)
* ```with_camera``` - whether the first camera is present (true or false).
* ```with_camera2``` - whether the second camera is present (true or false).
* ```publish_tf``` - whether to publish a simple TF tree corresponding to the centers of the RGB and depth cameras. Useful for debugging. 
* ```inverted``` - whether to publish the images from the first camera under the name of the second camera and viceversa. In the case when two cameras are connected, it's impossible to tell which is the first camera and which is the 2nd camera based on their IDs (they have the same ID); in that case by setting ```inverted:=yes``` the output of the two cameras will be inverted. 
 
For example, to start the first camera in the head_xtion namespace:

```roslaunch openni_wrapper with_camera:=true camera:=head_xtion with_camera2:=false publish_tf:=false```


Launching openni_wrapper on two computers
=========================================

To launch two instances of openni_wrapper, you have to set the ```STRANDS_COMPUTER_NAME``` environment variable to some value; this will be appended to the name of some of the nodes, allowing disambiguation at the roscore level. 

For example, on computer 1:

```
roslaunch openni_wrapper with_camera:=true camera:=chest_xtion with_camera2:=false
```

Or, if the first computer is also running the scitos robot drivers:
```
roslaunch scitos_bringup scitos.launch with_chest_camera:=true with_camera:=false
```

And on computer 2:

```
export STRANDS_COMPUTER_NAME=computer2
roslaunch openni_wrapper with_camera:=false with_camera2:=true camera:=head_xtion 
````
