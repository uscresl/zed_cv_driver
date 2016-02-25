ROS cuda-free driver for Stereolabs ZED
========================

zed_camera_node
------------------
This node uses [camera_info_manager](http://wiki.ros.org/camera_info_manager) for dealing with camera_info.
If no calibration data is set, it has dummy values except for width and height.

### Publish ###

* ~left/image_raw (sensor_msgs/Image)
* ~right/image_raw (sensor_msgs/Image)
* ~left/camera_info (sensor_msgs/CameraInfo)
* ~right/camera_info (sensor_msgs/CameraInfo)

### Service ###

* ~left/set_camera_info (sensor_msgs/SetCameraInfo)
* ~right/set_camera_info (sensor_msgs/SetCameraInfo)

### Parameters ###

* ~device_id (int: default 0) capture device id.
* ~frame_id (string: default "camera") frame_id of message header. Overwritten by left/right frame_ids
* ~left_frame_id
* ~right_frame_id
* ~left_camera_info_url (string) url of camera info yaml.
* ~right_camera_info_url (string) url of camera info yaml.

Nodelet
-------------------

This node works as nodelet (zed_cv_driver/ZedCvNodelet).
