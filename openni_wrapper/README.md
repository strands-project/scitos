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

* ```camera``` - the namespace of the camera (e.g. head_xtion)
 
For example, to start the first camera in the head_xtion namespace:

```roslaunch openni_wrapper camera:=head_xtion ```


Launching openni_wrapper on two computers
=========================================

For example, on computer 1:

```
roslaunch openni_wrapper main.launch camera:=chest_xtion 
```


And on computer 2:

```
roslaunch openni_wrapper camera:=head_xtion 
````
